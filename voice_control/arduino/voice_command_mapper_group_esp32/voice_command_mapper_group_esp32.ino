// ESP32-C3 Voice Command Mapper met groepen
// Doel:
// Elechouse Voice Recognition V3.1 record-ID omzetten naar robotcommando.
//
// Werking:
// 1. Bij opstarten wordt operator-groep geladen: records 28 t/m 33.
// 2. Operator zegt: Stijn, Merlijn, Rick of Jesse.
// 3. ESP32 laadt daarna de juiste persoonlijke groep.
// 4. Persoon zegt: Starten, Stop, Foutreset, OralB, Batterij, Dikke bout of Muurplug.
// 5. ESP32 print Robotcommando: ... naar USB Serial.
// 6. ROS2 voice_node.py leest dit en publiceert naar /voice_command.
//
// Belangrijk gedrag:
// - Stop blijft binnen dezelfde actieve persoon/groep.
// - Alleen Reset/Foutreset gaat terug naar de operatorgroep.
// - Daarna moet opnieuw Stijn, Merlijn, Rick of Jesse gekozen worden.
//
// Aansluitschema:
// Elechouse VCC -> ESP32 5V
// Elechouse GND -> ESP32 G
// Elechouse TXD -> ESP32 GPIO20
// Elechouse RXD -> ESP32 GPIO21

HardwareSerial voiceSerial(1);

#define VOICE_RX 20
#define VOICE_TX 21

// Elechouse protocol
#define FRAME_HEAD 0xAA
#define FRAME_END  0x0A
#define FRAME_CMD_LOAD 0x30
#define FRAME_CMD_CLEAR 0x31
#define FRAME_CMD_VR   0x0D

byte buffer[80];

enum ActiveGroup {
  GROUP_OPERATOR,
  GROUP_STIJN,
  GROUP_MERLIJN,
  GROUP_RICK,
  GROUP_JESSE
};

ActiveGroup activeGroup = GROUP_OPERATOR;

// Records per groep
byte recordsOperator[] = {28, 29, 30, 31, 32, 33};

byte recordsStijn[]   = {0, 1, 2, 3, 4, 5, 6};
byte recordsMerlijn[] = {7, 8, 9, 10, 11, 12, 13};
byte recordsRick[]    = {14, 15, 16, 17, 18, 19, 20};
byte recordsJesse[]   = {21, 22, 23, 24, 25, 26, 27};

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("Voice Command Mapper met groepen gestart");
  Serial.println("ESP32-C3 wacht op voice records...");

  voiceSerial.begin(9600, SERIAL_8N1, VOICE_RX, VOICE_TX);

  Serial.println("UART naar Elechouse gestart op 9600 baud");
  delay(500);

  loadOperatorGroup();
}

void loop() {
  if (voiceSerial.available()) {
    int length = readVoicePacket(buffer, sizeof(buffer));

    if (length > 0) {
      processVoicePacket(buffer, length);
    }
  }
}

void clearRecognizer() {
  byte clearCommand[] = {
    FRAME_HEAD,
    0x02,
    FRAME_CMD_CLEAR,
    FRAME_END
  };

  voiceSerial.write(clearCommand, sizeof(clearCommand));
  Serial.println("Recognizer geleegd");
  delay(300);

  while (voiceSerial.available()) {
    voiceSerial.read();
  }
}

// Laad een set records in de Elechouse recognizer
void loadRecords(byte *records, int count, const char *groupName) {
  clearRecognizer();
  
  byte loadCommand[16];

  loadCommand[0] = FRAME_HEAD;
  loadCommand[1] = count + 2;
  loadCommand[2] = FRAME_CMD_LOAD;

  for (int i = 0; i < count; i++) {
    loadCommand[3 + i] = records[i];
  }

  loadCommand[3 + count] = FRAME_END;

  voiceSerial.write(loadCommand, count + 4);

delay(300);

while (voiceSerial.available()) {
  voiceSerial.read();
}

  Serial.print("Voice records geladen voor groep: ");
  Serial.println(groupName);

  Serial.print("Records: ");
  for (int i = 0; i < count; i++) {
    Serial.print(records[i]);
    Serial.print(" ");
  }
  Serial.println();
  Serial.println();
}

void loadOperatorGroup() {
  activeGroup = GROUP_OPERATOR;
  loadRecords(recordsOperator, 6, "OPERATOR");
  Serial.println("Zeg nu: Stijn, Merlijn, Rick of Jesse");
}

void loadStijnGroup() {
  activeGroup = GROUP_STIJN;
  loadRecords(recordsStijn, 7, "STIJN");
  Serial.println("Groep Stijn actief");
  Serial.println("Zeg nu: Starten, Stop, Foutreset, OralB, Batterij, Dikke bout of Muurplug");
}

void loadMerlijnGroup() {
  activeGroup = GROUP_MERLIJN;
  loadRecords(recordsMerlijn, 7, "MERLIJN");
  Serial.println("Groep Merlijn actief");
  Serial.println("Zeg nu: Starten, Stop, Foutreset, OralB, Batterij, Dikke bout of Muurplug");
}

void loadRickGroup() {
  activeGroup = GROUP_RICK;
  loadRecords(recordsRick, 7, "RICK");
  Serial.println("Groep Rick actief");
  Serial.println("Zeg nu: Starten, Stop, Foutreset, OralB, Batterij, Dikke bout of Muurplug");
}

void loadJesseGroup() {
  activeGroup = GROUP_JESSE;
  loadRecords(recordsJesse, 7, "JESSE");
  Serial.println("Groep Jesse actief");
  Serial.println("Zeg nu: Starten, Stop, Foutreset, OralB, Batterij, Dikke bout of Muurplug");
}

// Robuust packet lezen op basis van lengte-byte.
// Dit is belangrijk, want record 10 is ook 0x0A.
// Daarom mag je niet stoppen bij de eerste 0x0A in het packet.
int readVoicePacket(byte *buf, int maxLen) {
  int index = 0;
  unsigned long startTime = millis();

  // Wacht op frame head 0xAA
  while (millis() - startTime < 1000) {
    if (voiceSerial.available()) {
      byte b = voiceSerial.read();

      if (b == FRAME_HEAD) {
        buf[index++] = b;
        break;
      }
    }
  }

  if (index == 0) {
    return 0;
  }

  // Lees length byte
  while (!voiceSerial.available()) {
    if (millis() - startTime > 1000) {
      return 0;
    }
  }

  byte lengthByte = voiceSerial.read();
  buf[index++] = lengthByte;

  int totalLength = lengthByte + 2;

  if (totalLength > maxLen) {
    return 0;
  }

  // Lees de rest van het packet
  while (index < totalLength) {
    if (voiceSerial.available()) {
      buf[index++] = voiceSerial.read();
    }

    if (millis() - startTime > 1000) {
      return 0;
    }
  }

  return index;
}

void processVoicePacket(byte *buf, int length) {
  Serial.print("Ontvangen packet: ");
  for (int i = 0; i < length; i++) {
    Serial.print("0x");
    if (buf[i] < 16) {
      Serial.print("0");
    }
    Serial.print(buf[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  if (length < 6) {
    Serial.println("Packet te kort.");
    return;
  }

  if (buf[0] != FRAME_HEAD) {
    Serial.println("Geen geldig packet: frame head klopt niet.");
    return;
  }

  if (buf[2] != FRAME_CMD_VR) {
    Serial.println("Geen voice-recognition packet.");
    return;
  }

  byte receivedId = buf[5];

  Serial.print("Ontvangen ID/index: ");
  Serial.println(receivedId);

  String action = mapVoiceToAction(receivedId);

  Serial.print("Actie: ");
  Serial.println(action);

  handleAction(action);
}

String mapVoiceToAction(byte id) {
  if (activeGroup == GROUP_OPERATOR) {
    return mapOperatorVoice(id);
  }

  if (activeGroup == GROUP_STIJN) {
    return mapPersonVoice(id, 0);
  }

  if (activeGroup == GROUP_MERLIJN) {
    return mapPersonVoice(id, 7);
  }

  if (activeGroup == GROUP_RICK) {
    return mapPersonVoice(id, 14);
  }

  if (activeGroup == GROUP_JESSE) {
    return mapPersonVoice(id, 21);
  }

  return "unknown";
}

String mapOperatorVoice(byte id) {
  // Werkt zowel als de module echte record-ID's teruggeeft,
  // als wanneer hij recognizer-index 0 t/m 5 teruggeeft.

  if (id == 28 || id == 0) {
    return "select_stijn";
  }

  if (id == 29 || id == 1) {
    return "select_merlijn";
  }

  if (id == 30 || id == 2) {
    return "select_rick";
  }

  if (id == 31 || id == 3) {
    return "select_jesse";
  }

  if (id == 32 || id == 4) {
    return "stop";
  }

  if (id == 33 || id == 5) {
    return "reset";
  }

  return "unknown";
}

String mapPersonVoice(byte id, byte groupStartRecord) {
  // Werkt zowel met echte record-ID's als met index 0 t/m 6.
  // Voor Stijn is groupStartRecord 0.
  // Voor Merlijn is groupStartRecord 7.
  // Voor Rick is groupStartRecord 14.
  // Voor Jesse is groupStartRecord 21.

  byte index;

  if (id >= groupStartRecord && id <= groupStartRecord + 6) {
    index = id - groupStartRecord;
  } else if (id <= 6) {
    index = id;
  } else {
    return "unknown";
  }

  switch (index) {
    case 0:
      return "start";

    case 1:
      return "stop";

    case 2:
      return "reset";

    case 3:
      return "pick_oral_b_head";

    case 4:
      return "pick_aaa_battery";

    case 5:
      return "pick_m6_bolt";

    case 6:
      return "pick_wall_plug";

    default:
      return "unknown";
  }
}

void handleAction(String action) {
  if (action == "select_stijn") {
    Serial.println("Operator heeft Stijn gekozen.");
    loadStijnGroup();
    return;
  }

  if (action == "select_merlijn") {
    Serial.println("Operator heeft Merlijn gekozen.");
    loadMerlijnGroup();
    return;
  }

  if (action == "select_rick") {
    Serial.println("Operator heeft Rick gekozen.");
    loadRickGroup();
    return;
  }

  if (action == "select_jesse") {
    Serial.println("Operator heeft Jesse gekozen.");
    loadJesseGroup();
    return;
  }

  if (action == "unknown") {
    Serial.println("WAARSCHUWING: onbekend record, geen robotactie uitvoeren.");
    return;
  }

  Serial.print("Robotcommando: ");
  Serial.println(action);

  // Alleen bij reset terug naar operator-keuze.
  // Stop blijft binnen dezelfde actieve persoon/groep.
  if (action == "reset") {
    delay(300);
    loadOperatorGroup();
  }
}
