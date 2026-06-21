#include <V2Base.h>
#include <V2Buttons.h>
#include <V2Device.h>
#include <V2LED.h>
#include <V2Link.h>
#include <V2MIDI.h>

V2DEVICE_METADATA("com.versioduo.spot", 1, "versioduo:samd:drum");

namespace {
  V2LED::WS2812 LED(5, PIN_LED_WS2812, &sercom2, SPI_PAD_0_SCK_1, PIO_SERCOM);
  V2Link::Port  Plug(&SerialPlug, PIN_SERIAL_PLUG_TX_ENABLE);
  V2Link::Port  Socket(&SerialSocket, PIN_SERIAL_SOCKET_TX_ENABLE);

  // Try to spread the power switching noise; run the timers with slightly
  // different periods, so they don't all start the rising edge of the PWM
  // period at the same time.
  std::array PWM{
    V2Base::Timer::PWM(0, 1000),
    V2Base::Timer::PWM(1, 1100),
    V2Base::Timer::PWM(2, 1200),
    V2Base::Timer::PWM(3, 1300),
    V2Base::Timer::PWM(4, 1400),
  };

  class {
  public:
    auto brightness() const -> float {
      return _brightness;
    }

    auto temperature() const -> float {
      return _temperature;
    }

    auto brightness(float b) {
      _brightness = std::powf(b, 2.2);
      update();
    }

    auto temperature(float t) {
      _temperature = t;

      const float boost{1.2f};
      _cold = std::clamp(_temperature * boost, 0.f, 1.f);
      _warm = std::clamp((1.f - _temperature) * boost, 0.f, 1.f);
      update();
    }

    auto reset() {
      brightness(0);
      temperature(0.5);
      update();
    }

  private:
    float _brightness{};
    float _temperature{0.5};
    float _warm{};
    float _cold{};

    auto update() -> void {
      duty(0, _brightness * _cold);
      duty(1, _brightness * _warm);
    }

    auto duty(uint8_t port, float duty) -> void {
      auto id{V2Base::Timer::PWM::getID(PIN_PULSE_CHANNEL + port)};
      PWM[id].setDuty(PIN_PULSE_CHANNEL + port, duty);
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

      // https://github.com/versioduo/arduino-board-package/blob/main/boards.txt
      usb.pid            = 0xe9b0;
      usb.ports.standard = 8;
    }

    auto allNotesOff() {
      Light.reset();
    }

  private:
    enum class CC {
      Brightness  = V2MIDI::CC::ChannelVolume,
      Temperature = V2MIDI::CC::Expression,
    };

    auto handleReset() -> void override {
      Light.reset();
    }

    auto handleControlChange(uint8_t channel, uint8_t controller, uint8_t value) -> void override {
      if (channel != 0)
        return;

      switch (controller) {
        case uint8_t(CC::Brightness):
          Light.brightness(float(value) / 127.f);
          break;

        case uint8_t(CC::Temperature):
          Light.temperature(float(value) / 127.f);
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
        JsonObject jsonController{jsonControllers.add<JsonObject>()};
        jsonController["name"]   = "Brightness";
        jsonController["number"] = uint8_t(CC::Brightness);
        jsonController["value"]  = uint8_t(Light.brightness() * 127.f);
      }
      {
        JsonObject jsonController{jsonControllers.add<JsonObject>()};
        jsonController["name"]   = "Temperature";
        jsonController["number"] = uint8_t(CC::Temperature);
        jsonController["value"]  = uint8_t(Light.temperature() * 127.f);
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

  class Button : public V2Buttons::Button {
  public:
    Button() : V2Buttons::Button(&_config, PIN_BUTTON) {}

  private:
    const V2Buttons::Config _config{.clickUsec{200 * 1000}, .holdUsec{500 * 1000}};

    auto handleClick(uint8_t count) -> void override {
      switch (count) {
        case 0:
          Device.allNotesOff();
          break;
      }
    }
  } Button;
}

auto setup() -> void {
  Serial.begin(9600);

  LED.begin();
  LED.setMaxBrightness(0.5);

  // Set the SERCOM interrupt priority, it requires a stable ~300 kHz interrupt
  // frequency. The call needs to be after begin().
  Link.begin();
  setSerialPriority(&SerialPlug, 2);
  setSerialPriority(&SerialSocket, 2);

  for (auto& p : PWM)
    p.begin();

  for (uint8_t p{PIN_PULSE_CHANNEL}; p < PIN_PULSE_CHANNEL + PIN_PULSE_CHANNEL_N; p++)
    V2Base::Timer::PWM::setupPin(p);

  digitalWrite(PIN_PULSE_ENABLE, HIGH);
  pinMode(PIN_PULSE_ENABLE, OUTPUT);

  Device.begin();
  Button.begin();
  Device.reset();
}

auto loop() -> void {
  LED.loop();
  MIDI.loop();
  Link.loop();
  V2Buttons::loop();
  Device.loop();

  if (Link.idle() && Device.idle())
    Device.sleep();
}
