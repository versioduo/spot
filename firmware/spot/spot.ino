#include <V2Base.h>
#include <V2Device.h>
#include <V2Link.h>
#include <V2MIDI.h>

V2DEVICE_METADATA("com.versioduo.spot", 4, "versioduo:samd:spot");

namespace {
  V2Link::Port Plug(&SerialPlug, PIN_SERIAL_PLUG_TX_ENABLE);
  V2Link::Port Socket(&SerialSocket, PIN_SERIAL_SOCKET_TX_ENABLE);

  // Try to spread the power switching noise; run the timers with slightly
  // different periods, so they don't all start the rising edge of the PWM
  // period at the same time.
  std::array PWM{
    V2Base::Timer::PWM(0, 8000),
    V2Base::Timer::PWM(1, 8100),
    V2Base::Timer::PWM(2, 8200),
    V2Base::Timer::PWM(3, 8300),
    V2Base::Timer::PWM(4, 8400),
  };

  class {
  public:
    auto brightness() const -> float {
      return _brightness;
    }

    auto brightness(float b) {
      _brightness = b;
      _warm       = 0;
      _cold       = 0;
      update();
    }

    auto channels(float cold, float warm) {
      _brightness = 0;
      _cold       = cold;
      _warm       = warm;
      update();
    }

    auto reset() {
      brightness(0);
      update();
    }

  private:
    float _brightness{};
    float _warm{};
    float _cold{};

    auto update() -> void {
      constexpr float gamma{2.2};
      constexpr float color{0.4};

      if (_brightness > 0.f) {
        duty(0, std::powf(_brightness, gamma + color));
        duty(1, std::powf(_brightness, gamma - color));
        return;
      }

      duty(0, std::powf(_cold, 2.2));
      duty(1, std::powf(_warm, 2.2));
    }

    auto duty(uint8_t port, float duty) -> void {
      auto id{V2Base::Timer::PWM::getID(PIN_PWM_CHANNEL + port)};
      PWM[id].setDuty(PIN_PWM_CHANNEL + port, duty);
    }
  } Light;

  class Device : public V2Device {
  public:
    Device() : V2Device() {
      metadata.vendor      = "Versio Duo";
      metadata.product     = "V2 spot";
      metadata.description = "LED Light";
      metadata.home        = "https://versioduo.com/#spot";

      system.download  = "https://versioduo.com/download";
      system.configure = "https://versioduo.com/configure";

      usb.ports.standard = 8;
    }

    auto allNotesOff() {
      _brightness.reset();
      _cold.reset();
      _warm.reset();
      Light.reset();

      _timeoutUsec = 0;
    }

  private:
    enum class CC {
      Brightness = V2MIDI::CC::ChannelVolume,
      Cold       = V2MIDI::CC::EffectControl1,
      Warm       = V2MIDI::CC::EffectControl2,
    };

    V2MIDI::CC::HighResolution<(uint8_t)V2MIDI::CC::ChannelVolume>  _brightness;
    V2MIDI::CC::HighResolution<(uint8_t)V2MIDI::CC::EffectControl1> _cold;
    V2MIDI::CC::HighResolution<(uint8_t)V2MIDI::CC::EffectControl2> _warm;
    uint32_t                                                        _timeoutUsec{};

    auto handleReset() -> void override {
      allNotesOff();
    }

    auto handleLoop() -> void override {
      if (_timeoutUsec == 0)
        return;

      if (V2Base::getUsecSince(_timeoutUsec) < 600 * 1000 * 1000)
        return;

      reset();
    }

    auto handleControlChange(uint8_t channel, uint8_t controller, uint8_t value) -> void override {
      _timeoutUsec = V2Base::getUsec();

      if (channel != 0)
        return;

      switch (controller) {
        case uint8_t(CC::Brightness):
        case V2MIDI::CC::ControllerLSB + (uint8_t)CC::Brightness:
          _brightness.setByte(controller, value);
          _warm.reset();
          _cold.reset();
          Light.brightness(_brightness.getFraction());
          break;

        case uint8_t(CC::Cold):
        case V2MIDI::CC::ControllerLSB + (uint8_t)CC::Cold:
          _brightness.reset();
          _cold.setByte(controller, value);
          Light.channels(_cold.getFraction(), _warm.getFraction());
          break;

        case uint8_t(CC::Warm):
        case V2MIDI::CC::ControllerLSB + (uint8_t)CC::Warm:
          _brightness.reset();
          _warm.setByte(controller, value);
          Light.channels(_cold.getFraction(), _warm.getFraction());
          break;

        case V2MIDI::CC::AllSoundOff:
        case V2MIDI::CC::AllNotesOff:
          allNotesOff();
          break;
      }
    }

    auto handleSystemReset() -> void override {
      reset();
    }

    auto exportInput(JsonObject json) -> void override {
      JsonArray jsonControllers{json["controllers"].to<JsonArray>()};
      {
        JsonObject j{jsonControllers.add<JsonObject>()};
        j["name"]      = "Brightness";
        j["number"]    = uint8_t(CC::Brightness);
        j["value"]     = _brightness.getMSB();
        j["valueFine"] = _brightness.getLSB();
      }
      {
        JsonObject j{jsonControllers.add<JsonObject>()};
        j["name"]      = "Cold";
        j["number"]    = uint8_t(CC::Cold);
        j["value"]     = _cold.getMSB();
        j["valueFine"] = _cold.getLSB();
      }
      {
        JsonObject j{jsonControllers.add<JsonObject>()};
        j["name"]      = "Warm";
        j["number"]    = uint8_t(CC::Warm);
        j["value"]     = _warm.getMSB();
        j["valueFine"] = _warm.getLSB();
      }
    }
  } Device;

  // Dispatch MIDI packets.
  class MIDI {
  public:
    auto loop() {
      if (!Device.usb.midi.receive(_midi))
        return;

      if (_midi.port == 0) {
        Device.dispatch(&Device.usb.midi, &_midi);

      } else {
        V2Link::Packet p(_midi.port - 1, _midi);
        p.midi.port = 0;
        Socket.send(p);
      }
    }

  private:
    V2MIDI::Packet _midi;
  } MIDI;

  // Dispatch Link packets.
  class Link : public V2Link {
  public:
    Link() : V2Link(&Plug, &Socket) {
      Device.link = this;
    }

  private:
    // Receive a host event from our parent device.
    auto receivePlug(V2Link::Packet& p) -> void override {
      if (p.type == V2Link::Packet::Type::MIDI)
        Device.dispatch(&Plug, &p.midi);
    }

    // Forward children device events to the host.
    auto receiveSocket(V2Link::Packet& p) -> void override {
      if (p.type == V2Link::Packet::Type::MIDI) {
        p.midi.port = p.address;
        Device.usb.midi.send(p.midi);
      }
    }
  } Link;
}

auto setup() -> void {
  Serial.begin(9600);

  // Set the SERCOM interrupt priority, it requires a stable ~300 kHz interrupt
  // frequency. The call needs to be after begin().
  Link.begin();
  setSerialPriority(&SerialPlug, 2);
  setSerialPriority(&SerialSocket, 1);

  for (auto& p : PWM)
    p.begin();

  for (uint8_t p{PIN_PWM_CHANNEL}; p < PIN_PWM_CHANNEL + PIN_PWM_CHANNEL_N; p++)
    V2Base::Timer::PWM::setupPin(p);

  Device.begin();
  Device.reset();
}

auto loop() -> void {
  MIDI.loop();
  Link.loop();
  Device.loop();

  if (Link.idle() && Device.idle())
    Device.sleep();
}
