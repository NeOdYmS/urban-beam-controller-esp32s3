# Measurement board design notes (AD8307 dual path)

## 1) Chaine de mesure

- Entree RF venant du coupleur directionnel:
  - Port FWD -> detecteur AD8307 #1
  - Port REV -> detecteur AD8307 #2
- Chaque voie AD8307:
  - attenuation / adaptation 50 ohms selon niveau attendu,
  - sortie log video,
  - filtre RC (ex: 10k + 100n) pour stabiliser ADC,
  - clamp de protection ADC (3V3 rail clamp + resistor series).

## 2) Alimentation

- Entree 5V depuis carte principale.
- LDO 3V3 analogique faible bruit (>= 150 mA).
- Decouplage local par voie AD8307:
  - 100 nF + 1 uF proche pin VCC.

## 3) Interface vers ESP32-S3

- `ADC_FWD` -> pin ADC forward du firmware.
- `ADC_REV` -> pin ADC reflected du firmware.
- GND analogique liee au plan GND principal en point unique proche ADC.

## 4) Recommandations layout RF

- Conserver pistes RF courtes, droites, references au plan GND.
- Isoler zone RF des moteurs/driver (bruit de commutation).
- Garder convertisseurs et stepper drivers hors carte mesure si possible.
- Prevoir connecteurs coaxiaux SMA de bonne qualite.

## 5) Calibration logicielle

- Mesurer sortie AD8307 sur charges connues.
- Construire table ou regression:
  - `Vf_volts -> P_fwd_dBm`
  - `Vr_volts -> P_ref_dBm`
- Convertir en SWR via coefficient de reflexion.
