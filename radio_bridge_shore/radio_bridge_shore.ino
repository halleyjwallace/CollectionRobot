// Shore-side USB <-> NRF24 radio bridge (second Arduino Uno, next to the laptop).
//
// MATLAB talks to this Uno exactly as it talks to the robot's Uno: it sends
// command lines ("LF,LR,RF,RR,pump" or "CAL") and gets the robot's replies
// back as the same text lines ("IMU,...", "PUMP ON", "FAILSAFE", ...).
//
// Each command line is sent as one radio packet to the boat's Raspberry Pi,
// which forwards it to the robot's Uno. The robot's replies come back in the
// radio acknowledgements (one packet late, i.e. ~50 ms at 20 Hz).
//
// Extra lines printed by the bridge itself:
//   BRIDGE READY       radio set up
//   RADIO HW MISSING   NRF24 not responding (check wiring / power)
//   RADIO OK           packets are being acknowledged by the boat
//   RADIO LOST         LOST_AFTER_FAILS packets in a row were not acknowledged
//
// Wiring (NRF24 via YL-105 adapter): VCC 5V, GND, CE D9, CSN D10,
// SCK D13, MOSI D11, MISO D12. Needs the "RF24" library by TMRh20.
#include <SPI.h>
#include <RF24.h>

RF24 radio(9, 10);                          // CE, CSN

// Must match pi_radio_relay.py on the boat
const uint8_t ADDRESS[6] = "BOAT1";
const uint8_t CHANNEL = 108;                // above most Wi-Fi channels
const int LOST_AFTER_FAILS = 5;             // ~0.25 s at 20 Hz

const int LINE_MAX = 33;                    // radio payload is at most 32 bytes
char line[LINE_MAX];
int lineLen = 0;
bool lineOverflow = false;

enum LinkState { LINK_UNKNOWN, LINK_OK, LINK_LOST };
LinkState linkState = LINK_UNKNOWN;
int failsInRow = 0;

// Turn an acknowledgement payload from the boat back into a text line
void printReply(const uint8_t *buf, uint8_t len) {
  if (len >= 21 && buf[0] == 'I') {
    float v[5];
    memcpy(v, buf + 1, sizeof(v));          // yaw rate, yaw, ax, ay, az
    Serial.print("IMU,");
    Serial.print(v[0], 2);
    Serial.print(',');
    Serial.print(v[1], 1);
    for (int i = 2; i < 5; i++) {
      Serial.print(',');
      Serial.print(v[i], 3);
    }
    Serial.println();
  } else if (len >= 2 && buf[0] == 'T') {
    Serial.write(buf + 1, len - 1);
    Serial.println();
  }
}

void sendLine(const char *text, uint8_t len) {
  if (radio.write(text, len)) {
    failsInRow = 0;
    if (linkState != LINK_OK) {
      linkState = LINK_OK;
      Serial.println("RADIO OK");
    }
    while (radio.available()) {
      uint8_t size = radio.getDynamicPayloadSize();
      if (size < 1 || size > 32) {
        radio.flush_rx();
        break;
      }
      uint8_t buf[32];
      radio.read(buf, size);
      printReply(buf, size);
    }
  } else if (++failsInRow >= LOST_AFTER_FAILS && linkState != LINK_LOST) {
    linkState = LINK_LOST;
    Serial.println("RADIO LOST");
  }
}

void setup() {
  Serial.begin(115200);
  if (!radio.begin()) {
    Serial.println("RADIO HW MISSING");
  }
  radio.setChannel(CHANNEL);
  radio.setDataRate(RF24_250KBPS);          // slowest = longest range
  radio.setPALevel(RF24_PA_LOW);            // raise to RF24_PA_HIGH for more range
  radio.enableDynamicPayloads();
  radio.enableAckPayload();
  radio.setRetries(2, 5);                   // 750 us between tries, 5 retries
  radio.openWritingPipe(ADDRESS);
  radio.stopListening();
  Serial.println("BRIDGE READY");
}

void loop() {
  // One command line from MATLAB -> one radio packet
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      if (!lineOverflow && lineLen > 0) {
        if (line[lineLen - 1] == '\r') {
          lineLen--;
        }
        sendLine(line, lineLen);
      }
      lineLen = 0;
      lineOverflow = false;
    } else if (lineLen < LINE_MAX - 1) {
      line[lineLen++] = c;
    } else {
      lineOverflow = true;
    }
  }
}
