# Zephyr patches

Patches to apply after `west update` wipes external modules.

## zephyr-lora-sx126x-standby.patch

**File:** `external/zephyr/drivers/lora/loramac_node/sx12xx_common.c`

**Problem:** The SX1262 wakeup from sleep hangs indefinitely. `modem_release()`
calls `Radio.Sleep()` after every operation, but the wakeup sequence in
`SX126xWakeup` can assert NSS while the chip's BUSY line is still HIGH from
the sleep context save, causing BUSY to get stuck permanently.

**Fix:** Replace `Radio.Sleep()` with `Radio.Standby()` in `modem_release` and
`sx12xx_ev_rx_done` so the chip stays in STDBY_RC between operations.

**Apply:**
```
cd external/zephyr
git apply ../../project/patches/zephyr-lora-sx126x-standby.patch
```

**Revert:**
```
cd external/zephyr
git checkout drivers/lora/loramac_node/sx12xx_common.c
```

---

## tracker.dts — E22-900M22S SX1262 required properties

These are **not patches** (they live in the project DTS, not external/zephyr),
but are documented here for reference.

The E22-900M22S module requires three DTS properties on the sx1262 node that
are easy to miss:

| Property | Value | Why |
|---|---|---|
| `dio2-tx-enable` | boolean | SX1262 drives DIO2 HIGH during TX to control the E22's internal RF switch. Without it the RF path stays in RX during TX and TxDone never fires. |
| `dio3-tcxo-voltage` | `SX126X_DIO3_TCXO_1V8` | E22 has an internal 32 MHz TCXO powered by DIO3 at 1.8 V. Without it the PLL has no stable reference, TX fails silently. |
| `tcxo-power-startup-delay-ms` | `5` | Startup time for the TCXO before calibration runs. |
