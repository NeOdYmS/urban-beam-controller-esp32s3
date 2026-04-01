# Carte mesure SWR inline – Design notes

PCB séparé, placé à côté de l'ESP32-S3.
S'insère dans la chaîne TX → antenne via BNC IN / BNC OUT.

## 1) Architecture

```
[TX/transceiver]
      |
    BNC IN ──┐
             [coupleur directionnel]
    BNC OUT ─┘        |            |
                   FWD tap      REV tap
                      |            |
                  [détecteur]  [détecteur]
                      |            |
                   V_fwd        V_rev
                      |            |
                  [pont résistif / calcul SWR]
                      |
                  sortie DC ──► SWR_ADC (GPIO 1 vers ESP32)
                      |
              [comparateur LM311]
                      |
              [prescaler 74HC4040 ÷256]
                      |
                  FREQ_OUT ──► FREQ_COUNTER (GPIO 2 vers ESP32 PCNT)
```

Le coupleur directionnel prélève une fraction du signal forward
et reflected sans perturber la chaîne TX → antenne.

## 2) Coupleur directionnel

- Type: transformateur bifilaire sur tore ferrite (FT37-43 ou similaire).
- Couplage ~20 dB pour puissances 1-100 W.
- Alternative: coupleur Stockton (2 tores + résistances).
- Impédance caractéristique: 50 Ω.
- BNC IN: entrée depuis le TX.
- BNC OUT: sortie vers l'antenne.

## 3) Détection SWR

Deux approches possibles:

### a) Diodes + pont résistif (design K6BEZ simple)
- 3× 50 Ω (1%), 2× diodes Schottky (1N5711, BAT46, AA143).
- Sortie FWD/REV redressée.
- MCP6002 en buffer ou sommateur pour produire une tension
  proportionnelle au SWR.
- Filtrage RC (10k + 100nF) avant sortie ADC.

### b) AD8307 dual path (log detector, meilleure dynamique)
- AD8307 #1 sur voie FWD, AD8307 #2 sur voie REV.
- Sortie log vidéo (25 mV/dB, 92 dB de dynamique).
- Différence des deux sorties → proportionnel au return loss.
- Filtrage RC avant ADC.

Dans les deux cas, la sortie est une tension DC unique envoyée
sur le GPI `SWR_ADC` (GPIO 1 de l'ESP32-S3).

## 4) Détection automatique de fréquence

L'ADC de l'ESP32-S3 est trop lent pour échantillonner du HF
(1.8-30 MHz) directement.

Solution: compteur d'impulsions matériel.

- **LM311** (comparateur): convertit le signal RF (tap FWD du coupleur)
  en signal carré 0/3.3V compatible logique.
- **74HC4040** (compteur binaire 12 bits): prescaler ÷256.
  - Sortie Q8 (÷256) → GPIO 2 de l'ESP32-S3.
  - Fréquence max en entrée: 30 MHz → sortie: ~117 kHz.
- L'ESP32-S3 utilise le périphérique **PCNT** (Pulse Counter) pour
  compter les fronts montants sur une fenêtre de 100 ms.
- Calcul: `freq_Hz = count × 256 / 0.1`
- Résolution: ~2.56 kHz (suffisant pour identifier la bande HF).

## 5) Interface vers ESP32-S3

Connecteur J3 (pin header 1×6, 2.54mm):

| Pin | Signal         | Direction | GPIO ESP32 |
|-----|----------------|-----------|------------|
| 1   | +5V            | → carte   | —          |
| 2   | GND            | commun    | GND        |
| 3   | SWR_ADC        | → ESP32   | GPIO 1     |
| 4   | FREQ_OUT (÷256)| → ESP32   | GPIO 2     |
| 5   | AGND           | commun    | —          |
| 6   | réservé        | —         | —          |

## 6) Alimentation

- 5V depuis carte ESP32 via J3 pin 1.
- Régulateur 3.3V local (LDO faible bruit) pour LM311 et logique.
- Découplage: 100nF + 1µF par IC.

## 7) Layout PCB

- Dimensions: ~60×40 mm.
- BNC IN et BNC OUT en bord de carte, côte à côte.
- Tore du coupleur au centre, pistes RF courtes.
- Zone analogique (détecteurs) séparée de zone numérique (74HC4040).
- Plan de masse continu sous les pistes RF.
- Connecteur J3 en bord opposé aux BNC.

## 8) Calibration logicielle

Utiliser des charges connues (50, 100, 150, 200 Ω) correspondant à
des SWR de 1, 2, 3, 4 dans un système 50 Ω.

- Mesurer la tension ADC pour chaque charge.
- Ajuster le mapping linéaire dans le firmware:
  `SWR = 1.0 + (adc_voltage / 3.3) × (SWR_MAX - 1.0)`
- Ou construire une LUT / regression si la réponse n'est pas linéaire.
