#ifndef BLE_RTES_H
#define BLE_RTES_H

#include "mbed.h"

// mbed BLE (mbed6)
#include "ble/BLE.h"
#include "ble/Gap.h"
#include "ble/gatt/GattService.h"
#include "ble/gatt/GattCharacteristic.h"
#include "ble/gap/AdvertisingDataBuilder.h"
#include "events/EventQueue.h"


extern events::EventQueue queue;

namespace rtes_ble {

static const UUID SERVICE_UUID("A0E1B2C3-D4E5-F6A7-B8C9-D0E1F2A3B4C5");
static const UUID TREMOR_UUID ("A1E2B3C4-D5E6-F7A8-B9C0-D1E2F3A4B5C6");
static const UUID DYS_UUID    ("A2E3B4C5-D6E7-F8A9-B0C1-D2E3F4A5B6C7");
static const UUID FOG_UUID    ("A3E4B5C6-D7E8-F9A0-B1C2-D3E4F5A6B7C8");


static const UUID TREMOR_E_UUID("B1E2B3C4-D5E6-F7A8-B9C0-D1E2F3A4B5C6");
static const UUID DYS_E_UUID   ("B2E3B4C5-D6E7-F8A9-B0C1-D2E3F4A5B6C7");
static const UUID FOG_I_UUID   ("B3E4B5C6-D7E8-F9A0-B1C2-D3E4F5A6B7C8");


static BLE &ble_if = BLE::Instance();
static bool connected = false;
static bool started = false;

static uint8_t tremor_flag = 0;
static uint8_t dys_flag    = 0;
static uint8_t fog_flag    = 0;

static float tremor_energy = 0.0f;
static float dys_energy    = 0.0f;
static float fog_index     = 0.0f;

static ReadOnlyGattCharacteristic<uint8_t> tremorChar(
    TREMOR_UUID, &tremor_flag,
    GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ |
    GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY
);

static ReadOnlyGattCharacteristic<uint8_t> dysChar(
    DYS_UUID, &dys_flag,
    GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ |
    GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY
);

static ReadOnlyGattCharacteristic<uint8_t> fogChar(
    FOG_UUID, &fog_flag,
    GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ |
    GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY
);


static ReadOnlyGattCharacteristic<float> tremorEnergyChar(
    TREMOR_E_UUID, &tremor_energy,
    GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ |
    GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY
);

static ReadOnlyGattCharacteristic<float> dysEnergyChar(
    DYS_E_UUID, &dys_energy,
    GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ |
    GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY
);

static ReadOnlyGattCharacteristic<float> fogIndexChar(
    FOG_I_UUID, &fog_index,
    GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ |
    GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY
);


static GattCharacteristic *chars[] = {
    &tremorChar,
    &dysChar,
    &fogChar,
    &tremorEnergyChar,
    &dysEnergyChar,
    &fogIndexChar
};

static GattService service(SERVICE_UUID, chars, sizeof(chars) / sizeof(chars[0]));

class ConnHandler : public ble::Gap::EventHandler {
public:
    void onConnectionComplete(const ble::ConnectionCompleteEvent &event) override {
        if (event.getStatus() == BLE_ERROR_NONE) {
            connected = true;
            printf("[BLE] Connected\r\n");
        }
    }

    void onDisconnectionComplete(const ble::DisconnectionCompleteEvent &event) override {
        (void)event;
        connected = false;
        printf("[BLE] Disconnected, restart advertising\r\n");
        ble_if.gap().startAdvertising(ble::LEGACY_ADVERTISING_HANDLE);
    }
};

static ConnHandler handler;


static void schedule_ble_events(BLE::OnEventsToProcessCallbackContext *context) {
    (void)context;
    queue.call(mbed::callback(&ble_if, &BLE::processEvents));
}


static void on_init_complete(BLE::InitializationCompleteCallbackContext *params) {
    if (params->error != BLE_ERROR_NONE) {
        printf("[BLE] init failed: %u\r\n", params->error);
        return;
    }

    ble_if.gattServer().addService(service);
    ble_if.gap().setEventHandler(&handler);

    static uint8_t adv_buf[ble::LEGACY_ADVERTISING_MAX_SIZE];
    ble::AdvertisingDataBuilder adv(adv_buf);

    adv.clear();
    adv.setFlags();
    adv.setName("RTES-PD"); 

    ble::AdvertisingParameters adv_params(
        ble::advertising_type_t::CONNECTABLE_UNDIRECTED,
        ble::adv_interval_t(ble::millisecond_t(100))
    );

    ble_if.gap().setAdvertisingParameters(ble::LEGACY_ADVERTISING_HANDLE, adv_params);
    ble_if.gap().setAdvertisingPayload(ble::LEGACY_ADVERTISING_HANDLE, adv.getAdvertisingData());
    ble_if.gap().startAdvertising(ble::LEGACY_ADVERTISING_HANDLE);

    started = true;
    printf("[BLE] Advertising started (name=RTES-PD)\r\n");
}


static inline void start() {
    if (started) return;
    ble_if.onEventsToProcess(schedule_ble_events);
    ble_if.init(on_init_complete);
}


static inline void update(bool isTremor, bool isDysk, bool isFog,
                          float tremorE, float dysE, float fogI) {
    if (!started) return;

  
    tremor_flag = isTremor ? 1 : 0;
    dys_flag    = isDysk   ? 1 : 0;
    fog_flag    = isFog    ? 1 : 0;

    tremor_energy = tremorE;
    dys_energy    = dysE;
    fog_index     = fogI;

   
    if (!connected) return;


    ble_if.gattServer().write(tremorChar.getValueHandle(), &tremor_flag, sizeof(tremor_flag));
    ble_if.gattServer().write(dysChar.getValueHandle(),    &dys_flag,    sizeof(dys_flag));
    ble_if.gattServer().write(fogChar.getValueHandle(),    &fog_flag,    sizeof(fog_flag));

    ble_if.gattServer().write(tremorEnergyChar.getValueHandle(),
                              reinterpret_cast<uint8_t*>(&tremor_energy), sizeof(tremor_energy));
    ble_if.gattServer().write(dysEnergyChar.getValueHandle(),
                              reinterpret_cast<uint8_t*>(&dys_energy), sizeof(dys_energy));
    ble_if.gattServer().write(fogIndexChar.getValueHandle(),
                              reinterpret_cast<uint8_t*>(&fog_index), sizeof(fog_index));
}

} 

#endif
