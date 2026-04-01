# Fusion 360 - carte complementaire de mesure SWR (debut)

Ce dossier contient un point de depart pour la carte analogique de mesure SWR basee sur AD8307, a raccorder a l'ESP32-S3 Ethernet.

## Contenu

- `urban_beam_measurement_start.sch`: squelette schematique Eagle/Fusion 360.
- `urban_beam_measurement_start.brd`: squelette board Eagle/Fusion 360 (80 x 60 mm).
- `measurement_design_notes.md`: architecture electronique recommandee.
- `measurement_bom_start.csv`: BOM de depart.

## Objectif de la carte complementaire

- Mesure puissance directe (Vf) et reflechie (Vr) via coupleur directionnel.
- Detection logarithmique via 2x AD8307.
- Filtrage analogique et protection ADC.
- Sortie analogique vers 2 ADC ESP32-S3.
- Alimentation 5V -> 3V3 faible bruit pour etage mesure.

## Import dans Fusion 360

1. Ouvrir Fusion 360 Electronics.
2. Importer `urban_beam_measurement_start.sch`.
3. Importer `urban_beam_measurement_start.brd`.
4. Completer les composants reels (librairies AD8307, SMA, op-amp, connecteurs).

## Notes

- Les fichiers `.sch/.brd` fournis ici sont un point de demarrage structurel.
- Les empreintes et regles RF doivent etre adaptees a ton atelier (impedance, stackup, largeur microstrip).
