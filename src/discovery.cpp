#include "discovery.h"
#include "log.h"
#include "commands.h"
#include "espmeshmesh.h"
#include "graph.h"

#if USE_ESP32
#include <esp_random.h>
#include <esp_mac.h>
#endif

#include <cstring>

namespace espmeshmesh {

#define INF_RSSI 1000
#define NULL_RSSI -1000

#define BEACONS_PERIOD 12
#define BEACONS_DELAY(X) (BEACONS_PERIOD / 2 + BEACONS_SLOT(X) * BEACONS_PERIOD)
#ifdef USE_ESP32
#define BEACONS_SLOT(X) (esp_random() % X)
#else
#define BEACONS_SLOT(X) (random() % X)
#endif

#define DISCCMD_RESET_TABLE_REQ 0x00
#define DISCCMD_RESET_TABLE_REP 0x01
#define DISCCMD_TABLE_SIZE_REQ 0x02
#define DISCCMD_TABLE_SIZE_REP 0x03
#define DISCCMD_TABLE_ITEM_GET_REQ 0x04
#define DISCCMD_TABLE_ITEM_GET_REP 0x05
#define DISCCMD_START_REQ 0x06
#define DISCCMD_START_REP 0x07
#define DISCCMD_BEACONS_SEND_REQ 0x08
#define DISCCMD_BEACONS_SEND_REP 0x09

static const char *TAG = "espmeshmesh.discovery";

Discovery::Discovery(EspMeshMesh *parent) : mParent(parent) {}

void Discovery::init() {
    if (mParent->getNodes()) {
        mParent->getNodes()->clear();
    }
}

void Discovery::loop() {
  uint32_t now = millis();
  if (mRunPhase == 1) {
    if (EspMeshMesh::elapsedMillis(now, mStartTime) > 50) {
      mRunPhase++;
      mStartTime = now;
    }
  } else if (mRunPhase == 2) {
    mStart.cmd1 = CMD_DISCOVERY_REQ;
    mStart.cmd2 = DISCCMD_BEACONS_SEND_REQ;

    mStartCompat.cmd1 = CMD_BEACONS_SEND;
    mStartCompat.filter = mStart.filter;
    mStartCompat.mask = mStart.mask;
    mStartCompat.slotnum = mStart.slotnum;

    mParent->broadCastSendData((uint8_t *) &mStart, sizeof(CmdStart_t));
    mRunPhase++;
  } else if (mRunPhase == 3) {
    if (EspMeshMesh::elapsedMillis(now, mStartTime) > 2000) {
      LIB_LOGD(TAG, "Discovery::loop discovery end");
      mRunPhase = 0;
      mStartTime = now;
      mParent->graphUpdated();
    }
  }

  if (mRunPhase == 11) {
    if (EspMeshMesh::elapsedMillis(now, mStartTime) > mBeaconDelay) {
      BaconsData_t data;
      data.reply1 = CMD_DISCOVERY_REQ;
      data.reply2 = DISCCMD_BEACONS_SEND_REP;
      data.id = Discovery::chipId();
      data.rssi = (int16_t) mParent->lastPacketRssi();

      mParent->uniCastSendData((uint8_t *) &data, sizeof(BaconsData_t), mParent->broadcastFromAddress());
      LIB_LOGD(TAG, "Discovery::loop beacon reply end");
      mRunPhase = 0;
    }
  }
}

void Discovery::process_beacon(uint32_t from, uint32_t id, int16_t rssi1, int16_t rssi2) {
  LIB_LOGD(TAG, "discovery_process_beacon from:%06lX id:%06lX rssi1:%d rssi2:%d\n", from, id, rssi1, rssi2);
  if (mParent->getNodes()) {
      mParent->getNodes()->add_edge(from, id, (rssi1 + rssi2) / 2);
  }
}

uint8_t Discovery::handle_frame(uint8_t *buf, uint16_t len, EspMeshMesh *parent) {
  uint8_t err = 1;
  switch (buf[0]) {
    case DISCCMD_START_REQ:
      if (len == sizeof(CmdStart_t) - 1) {
        if (mRunPhase == 0) {
          discoveryStart(buf, len);
          buf[0] = CMD_DISCOVERY_REP;
          buf[1] = DISCCMD_START_REP;
          parent->commandReply(buf, 2);
        }
        err = 0;
      }
      break;
    case DISCCMD_BEACONS_SEND_REQ:
      if (len == sizeof(CmdStart_t) - 1 && !isRunning()) {
        memcpy(((uint8_t *) &mStart) + 1, buf, sizeof(CmdStart_t) - 1);
        if (mStart.mask == 0 || (Discovery::chipId() & mStart.mask) == mStart.filter) {
          mRunPhase = 11;
          mStartTime = millis();
          mBeaconDelay = BEACONS_DELAY(mStart.slotnum);
          LIB_LOGD(TAG, "Discovery::handle_frame beacon reply id %02lX mask %d filt %d slots %d delay %ld",
                   Discovery::chipId() & mStart.mask, mStart.mask, mStart.filter, mStart.slotnum, mBeaconDelay);
        }
        err = 0;
      }
      break;
    case DISCCMD_BEACONS_SEND_REP:
      if (len == sizeof(BaconsData_t) - 1) {
        BaconsData_t data;
        memcpy(((uint8_t *) &data) + 1, buf, sizeof(BaconsData_t) - 1);
        process_beacon(parent->broadcastFromAddress(), data.id, data.rssi, (int16_t) parent->lastPacketRssi());
        err = 0;
      }
      break;

  }
  return err;
}

uint32_t Discovery::chipId() {
#ifdef IDF_VER
  uint64_t macAddress;
  esp_efuse_mac_get_default((uint8_t *) &macAddress);
  return (uint32_t) (macAddress & 0xFFFFFF);
#else
  return system_get_chip_id();
#endif
}

void Discovery::discoveryStart(uint8_t *buf, uint16_t len) {
  uint8_t *data = ((uint8_t *) &mStart) + 1;
  memcpy(data, buf, sizeof(CmdStart_t));
  discoveryStart(mStart.slotnum);
}

void Discovery::discoveryStart(uint8_t slotnum) {
  if (mRunPhase != 0)
    return;
  mStart.slotnum = slotnum;
  if (mStart.slotnum < 10)
    mStart.slotnum = 100;
  mStartTime = millis();
  mRunPhase = 1;
  LIB_LOGD(TAG, "Discovery::handle_frame discovery start %d", mStart.slotnum);
}

}  // namespace espmeshmesh
