# Arduino Uno and W25N01GV  Testing

**Step 1: Connection**

| Arduino Uno | W25N01GV |
| ----------- | -------- |
| 10 (SS)     | CS       |
| 11 (MOSI)   | DI       |
| 12 (MISO)   | DO       |
| 13 (CLK)    | CLK      |
| VCC         | WP       |
| VCC         | HOLD     |
| VCC         | VCC      |
| GND         | GND      |

**Step 2: Add Arduino Library:**

`nrf52832-ads1292-fw\arduino\WinbondW25N-0.2.2.zip`

**Step 3: Open file example and flashing and testing**

`nrf52832-ads1292-fw\arduino\W25N_WriteString\W25N_WriteString.ino`