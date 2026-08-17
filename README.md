# MEZZA_EXP_IN – Firmware di test scheda (STM32C552CE)

Firmware di collaudo per una scheda basata sul microcontrollore **STM32C552CE** (Cortex-M33), sviluppato con STM32CubeMX/CubeIDE e strutturato come progetto CMake standalone.

Lo scopo è verificare rapidamente, in fase di bring-up hardware, che gli 8 ingressi analogici della scheda funzionino correttamente, inviando le letture via UART in formato testuale leggibile su un terminale seriale.

## Cosa fa

- Legge in sequenza gli 8 canali ADC1 (`ADC1_IN0`…`ADC1_IN7`) collegati ai pin `PA0`…`PA7`.
- Interpreta ogni ingresso come un **loop di corrente 4-20 mA**, condizionato a monte in modo che:
  - `4 mA → 0.6 V`
  - `20 mA → 3.0 V`

  all'ingresso dell'ADC (riferimento `VREF+` esterno a 3.3 V).
- Calcola per ciascun canale sia il valore grezzo dell'ADC che la corrente corrispondente in mA.
- Invia il report testuale via **USART1** (`PA9` = TX, `PA10` = RX, 115200 baud, 8N1) ogni 500 ms circa.

Un valore fuori dal range 4–20 mA (es. 0 mA o valore massimo di fondo scala) è indicativo di un guasto sul canale (loop aperto o cortocircuito) ed è utile in fase di collaudo.

## Hardware

| Segnale | Pin (package) | Funzione |
|---|---|---|
| ADC1_IN0 | PA0 | Ingresso 4-20mA canale 1 |
| ADC1_IN1 | PA1 | Ingresso 4-20mA canale 2 |
| ADC1_IN2 | PA2 | Ingresso 4-20mA canale 3 |
| ADC1_IN3 | PA3 | Ingresso 4-20mA canale 4 |
| ADC1_IN4 | PA4 | Ingresso 4-20mA canale 5 |
| ADC1_IN5 | PA5 | Ingresso 4-20mA canale 6 |
| ADC1_IN6 | PA6 | Ingresso 4-20mA canale 7 |
| ADC1_IN7 | PA7 | Ingresso 4-20mA canale 8 |
| USART1_TX | PA9 | Uscita seriale (log testuale) |
| USART1_RX | PA10 | Ingresso seriale (non utilizzato) |

Riferimento ADC: `VREF+` esterno, 3.3 V.

## Struttura del progetto

Il progetto usa la nuova architettura STM32Cube (driver HAL/LL unificati per la famiglia STM32C5) generata da STM32CubeMX:

- `main.c` — codice applicativo (punto di ingresso, logica di lettura ADC e invio UART). È l'unico file pensato per essere modificato liberamente dall'utente.
- `generated/hal/` — codice di inizializzazione periferiche generato da CubeMX (`mx_*.c/h`), non va modificato a mano.
- `stm32c5xx_drivers/` — driver HAL/LL STM32C5.
- `user_modifiable/` — sorgenti di device/startup/linker script, editabili e uniti (merge) alle rigenerazioni CubeMX.
- `cmake/`, `CMakeLists.txt`, `CMakePresets.json` — configurazione build CMake/Ninja.

## Compilazione

Il progetto richiede **arm-none-eabi-gcc**, **CMake** e **Ninja**. Questi tool non sono necessariamente nel `PATH` di sistema: se è installato STM32CubeIDE, sono disponibili nei suoi plugin (es. `C:\ST\STM32CubeIDE_2.2.0\STM32CubeIDE\plugins\...\tools\bin`).

```sh
cmake --preset debug_GCC_STM32C552CEU6
cmake --build --preset debug_GCC_STM32C552CEU6
```

L'output (`.elf`, `.hex`, `.bin`) viene generato in `build/debug_GCC_STM32C552CEU6/`, pronto per il flashing tramite STM32CubeProgrammer, ST-Link Utility o STM32CubeIDE.

## Note

`build/` non è versionato (vedi `.gitignore`): viene rigenerato ad ogni compilazione.
