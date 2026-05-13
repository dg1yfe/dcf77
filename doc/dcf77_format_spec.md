# DCF77 Zeitzeichensignal — Formatbeschreibung für Decoder-Implementation

## Übersicht

| Parameter | Wert |
|---|---|
| Trägerfrequenz | 77,5 kHz |
| Standort | Mainflingen, Deutschland (50°01′N 9°00′E) |
| Reichweite | ~2000 km (Tagsignal), ~3500 km (Nachtsignal) |
| Zeitbasis | UTC + Offset (CET/CEST), ausgestrahlt von der PTB |
| Modulationsarten | Amplitudenmodulation (AM) + Phasenmodulation (PM, seit ~2012) |

---

## Teil 1: Amplitudenmodulation (AM)

### 1.1 Physikalisches Signal

Das AM-Signal kodiert 59 Bits pro Minute (Sekunde 0–58). An Sekunde 59 fehlt die Absenkung — dies dient als Minutenmarkierung.

**Pulsstruktur je Sekunde:**

```
Sekunde N:
├── 0 ms     : Absenkung der Amplitude auf ~15% (Pulsstart = Sekundenmarke)
├── 100 ms   : Wiederherstellung auf 100% → Bit-Wert = 0
├── 200 ms   : Wiederherstellung auf 100% → Bit-Wert = 1
└── 1000 ms  : Nächste Sekundenmarke beginnt
```

**Ausnahme Sekunde 59:** Keine Absenkung → Minutenmarkierung. Der nächste Puls markiert Sekunde 0 der neuen Minute.

**Dekodierstrategie:** Die Breite der Amplitudenabsenkung bestimmt den Bitwert:
- Pulsbreite ≤ ~150 ms → Bit = 0
- Pulsbreite > ~150 ms → Bit = 1
- Keine Absenkung → Minutenmarkierung (Sekunde 59)

**Timing-Toleranz:** Die PTB-Spezifikation gibt ±0,1 ms als Genauigkeit der Pulsbreite an. Ein Decoder sollte ±10 ms Toleranz einplanen.

---

### 1.2 Bit-Rahmenstruktur (59 Bits pro Minute)

Alle BCD-Felder werden **LSB-first** übertragen (niedrigstwertiges Bit zuerst).

```
Bit  Bedeutung                      Mögliche Werte / Hinweise
─────────────────────────────────────────────────────────────────────
  0  Start-Bit (Minutenmarkierung)  Immer 0
─────────────────────────────────────────────────────────────────────
 1–14  Zivilschutzbits / Wetterdaten  Nicht öffentlich spezifiziert
        (interne PTB-Nutzung)         Für Decoder: ignorieren oder speichern
─────────────────────────────────────────────────────────────────────
 15  Call-Bit                        0 = Normalbetrieb
                                     1 = Anomalie am Sender
─────────────────────────────────────────────────────────────────────
 16  Sommerzeitankündigung           1 = Zeitumstellung in der nächsten Stunde
─────────────────────────────────────────────────────────────────────
 17  MESZ aktiv (UTC+2)             1 = MESZ (Sommerzeit)
 18  MEZ aktiv  (UTC+1)             1 = MEZ (Winterzeit)
      Invariante: Bit17 XOR Bit18 = 1 (genau eines ist gesetzt)
─────────────────────────────────────────────────────────────────────
 19  Schaltsekunden-Ankündigung     1 = Schaltsekunde am Ende dieser Stunde
─────────────────────────────────────────────────────────────────────
 20  Start Zeitinformation          Immer 1 (Synchronisationsmarke)
─────────────────────────────────────────────────────────────────────
     MINUTEN (BCD, 00–59):
 21  1-Minuten-Bit   (Wert 1)
 22  2-Minuten-Bit   (Wert 2)
 23  4-Minuten-Bit   (Wert 4)
 24  8-Minuten-Bit   (Wert 8)
 25  10-Minuten-Bit  (Wert 10)
 26  20-Minuten-Bit  (Wert 20)
 27  40-Minuten-Bit  (Wert 40)
─────────────────────────────────────────────────────────────────────
 28  Paritätsbit Minuten            Gerades Paritätsbit über Bits 21–27
─────────────────────────────────────────────────────────────────────
     STUNDEN (BCD, 00–23):
 29  1-Stunden-Bit   (Wert 1)
 30  2-Stunden-Bit   (Wert 2)
 31  4-Stunden-Bit   (Wert 4)
 32  8-Stunden-Bit   (Wert 8)
 33  10-Stunden-Bit  (Wert 10)
 34  20-Stunden-Bit  (Wert 20)
─────────────────────────────────────────────────────────────────────
 35  Paritätsbit Stunden            Gerades Paritätsbit über Bits 29–34
─────────────────────────────────────────────────────────────────────
     TAG DES MONATS (BCD, 01–31):
 36  1er-Bit   (Wert 1)
 37  2er-Bit   (Wert 2)
 38  4er-Bit   (Wert 4)
 39  8er-Bit   (Wert 8)
 40  10er-Bit  (Wert 10)
 41  20er-Bit  (Wert 20)
─────────────────────────────────────────────────────────────────────
     WOCHENTAG (BCD, 1=Montag … 7=Sonntag):
 42  1er-Bit   (Wert 1)
 43  2er-Bit   (Wert 2)
 44  4er-Bit   (Wert 4)
─────────────────────────────────────────────────────────────────────
     MONAT (BCD, 01–12):
 45  1er-Bit   (Wert 1)
 46  2er-Bit   (Wert 2)
 47  4er-Bit   (Wert 4)
 48  8er-Bit   (Wert 8)
 49  10er-Bit  (Wert 10)
─────────────────────────────────────────────────────────────────────
     JAHR (BCD, 00–99, zweistellig):
 50  1er-Bit   (Wert 1)
 51  2er-Bit   (Wert 2)
 52  4er-Bit   (Wert 4)
 53  8er-Bit   (Wert 8)
 54  10er-Bit  (Wert 10)
 55  20er-Bit  (Wert 20)
 56  40er-Bit  (Wert 40)
 57  80er-Bit  (Wert 80)
─────────────────────────────────────────────────────────────────────
 58  Paritätsbit Datum              Gerades Paritätsbit über Bits 36–57
─────────────────────────────────────────────────────────────────────
 59  Minutenmarkierung              Kein Puls — nur implizit als Lücke
─────────────────────────────────────────────────────────────────────
```

### 1.3 BCD-Dekodierung

```
Wert = bit[n+0]×1 + bit[n+1]×2 + bit[n+2]×4 + bit[n+3]×8
     + bit[n+4]×10 + bit[n+5]×20 + ...
```

**Beispiel Minuten (Bits 21–27):**
```
Bits: [0, 1, 1, 0, 1, 0, 0]  (LSB-first)
Wert: 0×1 + 1×2 + 1×4 + 0×8 + 1×10 + 0×20 + 0×40 = 16 Minuten
```

### 1.4 Paritätsprüfung

**Gerades Paritätsbit:** Die Anzahl der 1-Bits im geschützten Bereich *einschließlich* des Paritätsbits ist immer gerade.

```
P28 = (bit21 XOR bit22 XOR ... XOR bit27)          // Minuten
P35 = (bit29 XOR bit30 XOR ... XOR bit34)          // Stunden
P58 = (bit36 XOR bit37 XOR ... XOR bit57)          // Datum komplett
```

**Plausibilitätstest:** Nach Dekodierung prüfen:
- `00 ≤ Minuten ≤ 59`
- `00 ≤ Stunden ≤ 23`
- `01 ≤ Tag ≤ 31`
- `01 ≤ Monat ≤ 12`
- `00 ≤ Jahr ≤ 99`
- `01 ≤ Wochentag ≤ 07`
- `bit[0] == 0` und `bit[20] == 1`
- `bit[17] XOR bit[18] == 1`

### 1.5 Übertragenes Zeitmodell

Die übertragene Zeit ist die **lokale Zeit Mitteleuropas** (MEZ/MESZ), nicht UTC. Zur Konvertierung in UTC:
- MEZ  (Bit 18 = 1): UTC = Lokalzeit − 1h
- MESZ (Bit 17 = 1): UTC = Lokalzeit − 2h

**Wichtig:** Die übertragene Zeit ist die Zeit der **nächsten vollen Minute** (d. h. der Minutenbeginn, der mit der Minutenmarkierung an Sekunde 59 eintritt). Der Rahmen, der gerade empfangen wird, kodiert die Zeit der Minute, in der er *endet*.

---

## Teil 2: Phasenmodulation (PM)

> ⚠️ **Hinweis zur Quelllage:** Die PTB hat die PM-Spezifikation nicht vollständig öffentlich dokumentiert. Die folgenden Angaben basieren auf veröffentlichten Forschungsarbeiten (insbesondere Hetzel et al., PTB) und rekonstruiertem Wissen. Wo Unsicherheit besteht, ist dies gekennzeichnet.

### 2.1 Einführung und Motivation

Die Phasenmodulation wurde ab **ca. 2012/2013** eingeführt. Sie überträgt dieselbe Zeitinformation wie die AM, ist aber robuster gegenüber Amplitudenstörungen (Impulsnoise, Einstrahlungen). Insbesondere für Softwareempfänger (SDR) bietet das PM-Signal bessere Dekodierbarkeit bei schwachem Signal.

### 2.2 Physikalisches Modulationsprinzip

Die Phasenmodulation ist ein **BPSK-Spreizspektrumsignal** (Binary Phase Shift Keying), das auf den 77,5-kHz-Träger aufgeprägt wird. Die AM-Absenkungen bleiben erhalten und überlagern sich mit dem PM-Signal.

**Phasenzustände:**
```
Chip = 0 → Trägerphase +φ  (ca. +11° bis +13°, PTB-Quelle nicht eindeutig)
Chip = 1 → Trägerphase −φ
```
Der genaue Phasenwinkel φ ist in der öffentlichen Literatur nicht einheitlich angegeben.
⚠️ *Wahrscheinlichster Wert: φ ≈ 11,3° (entspricht einem Pegel-Seitenband von −15 dB). Decoder sollten φ als Kalibrationsparameter behandeln.*

### 2.3 Chip-Sequenz pro Sekunde

Jede Sekunde enthält eine **512-Chip-Pseudozufallssequenz** (PRBS = Pseudo-Random Binary Sequence).

```
Chip-Rate:     512 Chips/s  → Chipdauer = ~1,953 ms pro Chip
Sequenzlänge:  512 Chips pro Sekunde
```

⚠️ *Die Chip-Rate 512 Chips/s ist in mehreren unabhängigen Quellen angegeben, aber die PTB hat keine normative Spezifikation veröffentlicht. Alternative Angaben (z. B. 1000 Chips/s) sind in der Literatur vorhanden.*

### 2.4 Sequenzgenerierung (LFSR-Basissequenz)

Die Basissequenz ist eine **m-Sequenz** (maximale Länge), erzeugt durch ein lineares rückgekoppeltes Schieberegister (LFSR).

```
LFSR-Länge:  9 Bit  → Sequenzlänge = 2⁹ − 1 = 511 Chips
```

Das Generatorpolynom ist laut publizierten Implementierungen:
```
p(x) = x⁹ + x⁴ + 1
```

⚠️ *Dieses Polynom ist aus rekonstruierten Implementierungen (Open-Source-Decodern) entnommen, nicht aus einer offiziellen PTB-Quelle.*

**LFSR-Pseudocode:**
```python
def lfsr_step(state: int) -> tuple[int, int]:
    """9-Bit LFSR, Taps an Position 9 und 4 (1-indiziert)."""
    bit = ((state >> 8) ^ (state >> 3)) & 1   # XOR Bit 9, Bit 4
    state = ((state << 1) | bit) & 0x1FF      # 9-Bit maske
    return state, bit

def generate_m_sequence(seed: int = 1) -> list[int]:
    chips = []
    state = seed
    for _ in range(511):
        state, chip = lfsr_step(state)
        chips.append(chip)
    return chips  # 511 Chips, letzter Chip wird wiederholt → 512
```

### 2.5 Zeitcodierung in der PM-Sequenz

Die gesendete Chip-Sequenz pro Sekunde ist nicht die rohe m-Sequenz, sondern das Ergebnis einer **XOR-Verknüpfung** der m-Sequenz mit einem zeitabhängigen Code-Wort.

```
TX_Sequenz[s] = m_Sequenz XOR Code_Wort[s]
```

Das Code-Wort `Code_Wort[s]` kodiert die Sekunde `s` innerhalb der Minute sowie die vollständige AM-Zeitinformation (dieselben Bits wie im AM-Rahmen).

⚠️ *Die genaue Konstruktion des Code-Worts (welche Bits an welchen Chip-Positionen, Faltungscode vs. direkte Abbildung) ist aus öffentlichen Quellen **nicht eindeutig rekonstruierbar**. Folgende Struktur ist in der Gemeinschaft bekannt, aber nicht normativ bestätigt:*

```
Für Sekunde s (0–58) der Minute:
  - Chips 0–8:   kodieren den Sekundenindex s (9 Bit binär)
  - Chips 9–N:   kodieren die AM-Zeitbits (s. Teil 1) in noch
                  nicht vollständig öffentlich belegter Reihenfolge
```

**Empfehlung für Implementierung:** Für einen funktionsfähigen PM-Decoder existieren Open-Source-Referenzimplementierungen (z. B. `dcf77_metronome` von Udo Klein, `thijzert/dcf77` auf GitHub), die als Referenz dienen sollten, bis eine offizielle PTB-Spezifikation vorliegt.

### 2.6 PM-Dekodierungsstrategie

```
1. Quadraturdemodulation des 77,5-kHz-Trägers (I/Q)
2. Carrier-Phase-Recovery (PLL oder Hilbert-Transform)
3. Chip-Synchronisation: Korrelation mit bekannter m-Sequenz
   → Liefert Timing-Offset und Chip-Grenze
4. BPSK-Entscheidung: sign(Re(chip_correlator)) → 0 oder 1
5. XOR der empfangenen Sequenz mit der lokalen m-Sequenz
   → Ergibt das gesendete Code-Wort
6. Code-Wort dekodieren → Zeitinformation
```

**Vorteil für Decoder:** Schritt 3 (Korrelation mit m-Sequenz) ermöglicht Zeitstempel-Genauigkeit im Sub-Millisekunden-Bereich, unabhängig von der AM.

---

## Teil 3: Zusammenfassung für Decoder-Architektur

### 3.1 Signalfluss AM-Decoder

```
Rohsignal (77,5 kHz)
  → Amplitudenhüllkurve extrahieren (Gleichrichter + Tiefpass)
  → Schwellwertdetektion: Absenkung ja/nein
  → Pulsdauermessung: 100 ms / 200 ms / kein Puls
  → Bit-Buffer[0..58] befüllen
  → Minutenmarkierung erkennen (fehlendes Bit 59)
  → Rahmen dekodieren (BCD + Parität)
  → Zeitstempel ausgeben
```

### 3.2 Signalfluss PM-Decoder

```
Rohsignal (77,5 kHz)
  → I/Q-Demodulation (PLL)
  → Chip-Synchronisation (m-Sequenz-Korrelation)
  → BPSK-Entscheidung je Chip
  → XOR mit m-Sequenz → Code-Wort
  → Code-Wort → Zeitstempel dekodieren
  → Zeitstempel ausgeben (höhere Präzision als AM)
```

### 3.3 Kombinierter Decoder

Ein robuster Decoder kombiniert beide:
- **AM** für einfache Implementierung und Minutensynchronisation
- **PM** für Sub-Millisekunden-Präzision und Störfestigkeit
- Gegenseitige Plausibilitätsprüfung (AM- und PM-Ergebnis müssen übereinstimmen)

---

## Teil 4: Bekannte Einschränkungen und Fallstricke

| Problem | Beschreibung | Gegenmittel |
|---|---|---|
| Signalunterbrechungen | AM-Puls kann durch Störungen fehlen | Mehrere Minuten akkumulieren, Mehrheitsentscheid |
| Sommerzeit-Umschaltung | In der Umschaltstunde sind Bits 17 & 16 gesetzt | Bit 16 prüfen, Umschaltung explizit behandeln |
| Schaltsekunden | Sekunde 60 existiert, kein Puls an Sekunde 59 entfällt | Bit 19 im Voraus prüfen |
| Jahrtausend | Nur 2-stelliges Jahr übertragen | Fahrzeugbereich: 2000–2099 annehmen |
| PM-Code unbekannt | Code-Wort-Konstruktion nicht normativ | Open-Source-Referenzen verwenden |

---

## Quellen und weiterführende Referenzen

- **PTB DCF77-Seite:** https://www.ptb.de/cms/de/ptb/fachabteilungen/abt4/fb-44/ag-442/verbreitung-der-gesetzlichen-zeit/dcf77.html
- **Wikipedia DCF77 (EN):** https://en.wikipedia.org/wiki/DCF77  (AM-Rahmenstruktur gut dokumentiert)
- **Udo Klein (dcf77.de):** Detaillierte PM-Analyse und Open-Source-Arduino-Decoder
- **GitHub: thijzert/dcf77:** Referenz-PM-Decoder mit m-Sequenz-Implementierung
- **Hetzel et al. (PTB-Bericht):** Veröffentlichungen zur Phasenmodulations-Einführung (Metrologia)
