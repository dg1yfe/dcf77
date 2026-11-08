# DCF77 Zeitzeichensignal — Formatbeschreibung für Decoder-Implementation

## Übersicht

| Parameter | Wert |
|---|---|
| Trägerfrequenz | 77,5 kHz |
| Standort | Mainflingen, Deutschland (50°01′N 9°00′E) |
| Reichweite | ~2000 km (Tagsignal), ~3500 km (Nachtsignal) |
| Zeitbasis | UTC + Offset (CET/CEST), ausgestrahlt von der PTB |
| Modulationsarten | Amplitudenmodulation (AM) + Phasenmodulation (PM, seit den 1980er Jahren) |

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

> ℹ️ **Hinweis zur Quelllage:** Die PM ist in Hetzel (1988, EFTF) beschrieben; eine Zusammenfassung steht in [`hetzel_1988_eftf_summary.md`](hetzel_1988_eftf_summary.md). Die dort beschriebenen Parameter sind die Referenz für diese Spezifikation und stimmen mit RTL und Python-Modellen dieses Repos überein. Das aktuelle Betriebsverhalten des Senders (Phasenhub) ist nicht normativ veröffentlicht und ist als Kalibrationsparameter zu behandeln.

### 2.1 Einführung und Motivation

Die Phasenmodulation wird seit den **1980er Jahren** ausgesendet (Hetzel 1987/1988; Messungen bereits 1987). Sie ist eine pseudozufällige Phasenumtastung (PRPSK) des Trägers und überträgt dieselbe Zeitinformation wie die AM, ist aber robuster gegenüber Amplitudenstörungen (Impulsnoise, Einstrahlungen). Der Phasenhub ist klein und im Mittel null, sodass der Träger weiterhin als Frequenznormal nutzbar bleibt. Für Softwareempfänger (SDR) bietet das PM-Signal bessere Dekodierbarkeit bei schwachem Signal und eine deutlich schärfere Zeitmarke als die AM-Flanke.

### 2.2 Physikalisches Modulationsprinzip

Die Phasenmodulation ist ein **BPSK-Spreizspektrumsignal** (Binary Phase Shift Keying), das auf den 77,5-kHz-Träger aufgeprägt wird. Die AM-Absenkungen bleiben erhalten und überlagern sich mit dem PM-Signal. Alle Phasensprünge und AM-Flanken liegen auf positiven Nulldurchgängen des Trägers.

**Phasenzustände (nach der SIK-XOR-Verknüpfung):**
```
Chip = 0 → Trägerphase −φ
Chip = 1 → Trägerphase +φ
```
Das Vorzeichen bezieht sich auf die HF-Phase des empfangenen Trägers (Phasenvoreilung positiv). Es ist an echten Aufnahmen verifiziert: eine unabhängige Python-Demodulation von `dcf_strong` und der RTL-Empfänger stimmen darin überein. Hetzel (1988) gibt „Ausgang low → φm + Δφ" an; der Unterschied liegt vermutlich in der Phasendefinition (Phasenzeit bzw. Verzögerung statt Phasenwinkel).

Der RTL-Empfänger liefert die konjugierte Phase: Der Vorzeichen-Mischer bildet Q = x·sgn(sin ωt) statt −x·sgn(sin ωt), die CORDIC meldet daher −φ (`dcf77_core` kompensiert mit −cordic_phase für die PLL). Intern gilt deshalb Chip 0 → positiver Phasenfehler.

Der Phasenhub φ betrug in der Veröffentlichung von 1988 **±10°**; im heutigen Betrieb sind bis zu **±15°** gebräuchlich (die Simulation dieses Repos verwendet 15,6°, an `dcf_strong` wurden ca. 17° gemessen). Decoder sollten φ als Kalibrationsparameter behandeln; das Vorzeichen-Mapping ist maßgeblich, nicht der Betrag.

### 2.3 Chip-Sequenz pro Sekunde

Jede Sekunde trägt **eine 512-Chip-Pseudozufallssequenz** (PRBS), die zugleich **genau ein Datenbit** (Sequence Inversion Keying, SIK) überträgt: Das Bit ist über alle 512 Chips verteilt.

```
Chip-Rate:     77 500 / 120 = 645,83 Chips/s  (120 Trägerperioden je Chip)
Chipdauer:     ≈ 1,548 ms
Sequenzlänge:  512 Chips  → ≈ 793 ms
Startzeit:     200 ms nach Sekundenbeginn (AM-Flanke)
Ende:          ≈ 7 ms vor der nächsten Sekundenmarke
```

Die Sequenz belegt nur den Teil der Sekunde, in dem die Trägeramplitude **nicht** abgesenkt ist (nach dem bis zu 200 ms langen AM-Puls); daher ist die Chipdauer 1,548 ms und nicht 1 s / 512.

### 2.4 Sequenzgenerierung (LFSR-Basissequenz)

Die Basissequenz ist eine **m-Sequenz** maximaler Länge, erzeugt durch ein 9-stufiges Fibonacci-LFSR:

```
LFSR-Länge:  9 Bit  → Periode 2⁹ − 1 = 511 Chips
Rückkopplung: XOR der Stufen 5 und 9  → p(x) = x⁹ + x⁵ + 1
Startwert:   "000010001" (0x011), Ausgabe der Stufe 9 vor dem Schieben
```

Das reziproke Polynom x⁹ + x⁴ + 1 erzeugt die **zeitlich umgekehrte** Sequenz; sie korreliert mit der gesendeten Sequenz nur schwach (verifiziert: max. |Kreuzkorrelation| 45 von 511) und ist daher **nicht austauschbar**.

**Erweiterung auf 512 Chips:** Der Sender stellt der 511-Chip-m-Sequenz **ein Chip „0" voran** (keine Wiederholung eines Chips). Dadurch hat die Sequenz genau 256 Nullen und 256 Einsen, die Phasensumme über die Sequenz ist **null** (wichtig: kein DC-Anteil, der Träger bleibt phasenrein).

```python
def make_prbs512() -> list[int]:
    state = 0b000010001
    chips = [0]                              # vorangestelltes Null-Chip
    for _ in range(511):
        chips.append((state >> 8) & 1)       # Stufe 9, vor dem Schieben
        fb = ((state >> 4) ^ (state >> 8)) & 1   # Stufen 5 XOR 9
        state = ((state << 1) | fb) & 0x1FF
    return chips                             # 256 Nullen, 256 Einsen
```

### 2.5 Datencodierung (Sequence Inversion Keying)

Pro Sekunde wird **ein Bit** übertragen: Bit = 0 sendet die Sequenz unverändert, Bit = 1 sendet die invertierte Sequenz.

```
TX_Sequenz[s] = PRBS512 XOR bit[s]
```

Die Datenbits entsprechen den AM-Bits (Teil 1). Zur Minutenmarkierung tragen die **Sekunden 0–9** die invertierte Sequenz; Sekunde 59 wird – anders als bei der AM – nicht ausgelassen. Eine zusätzliche Chip-weise Zeit- oder Sekundenindex-Codierung gibt es nicht.

### 2.6 PM-Dekodierungsstrategie

```
1. Quadraturdemodulation des 77,5-kHz-Trägers (I/Q)
2. Carrier-Phase-Recovery (PLL), Phasenfehler als Eingang
3. Synchronisation: Korrelation mit der bekannten 512-Chip-Sequenz
   → Liefert Chip-Phase / Sekundenanfang (Korrelationsspitze ≈ 2 Chips breit)
4. Bit-Entscheidung: Vorzeichen der Gesamtkorrelation pro Sekunde
   → positiv = Bit 0, negativ = Bit 1
5. Bits der Sekunden 0..58 wie im AM-Rahmen dekodieren (Teil 1)
```

**Vorteil für Decoder:** Schritt 3 (Korrelation mit der Sequenz) ermöglicht Zeitstempel-Genauigkeit im Sub-Millisekunden-Bereich, unabhängig von der AM.

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
  → Bit-Entscheidung per Vorzeichen der Sequenz-Korrelation (SIK)
  → Bits wie im AM-Rahmen zum Zeitstempel dekodieren
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
| Falsche Sequenzrichtung | x⁹+x⁴+1 erzeugt die zeitlich umgekehrte Sequenz | Taps 5 & 9 (x⁹+x⁵+1), vorangestelltes Null-Chip |

---

## Quellen und weiterführende Referenzen

- **PTB DCF77-Seite:** https://www.ptb.de/cms/de/ptb/fachabteilungen/abt4/fb-44/ag-442/verbreitung-der-gesetzlichen-zeit/dcf77.html
- **Wikipedia DCF77 (EN):** https://en.wikipedia.org/wiki/DCF77  (AM-Rahmenstruktur gut dokumentiert)
- **Udo Klein (dcf77.de):** Detaillierte PM-Analyse und Open-Source-Arduino-Decoder
- **GitHub: thijzert/dcf77:** Referenz-PM-Decoder mit m-Sequenz-Implementierung
- **Hetzel, P. (1988):** Proc. 2nd EFTF, S. 351–364; [PDF bei PTB](https://www.ptb.de/cms/fileadmin/internet/fachabteilungen/abteilung_4/4.4_zeit_und_frequenz/pdf/5_1988_Hetzel_-_Proc_EFTF_88.pdf), Zusammenfassung: [`hetzel_1988_eftf_summary.md`](hetzel_1988_eftf_summary.md)
- **Hetzel, P. (1987):** Dissertation, Univ. Stuttgart; [PDF bei PTB](https://www.ptb.de/cms/fileadmin/internet/fachabteilungen/abteilung_4/4.4_zeit_und_frequenz/pdf/Hetzel_1987_Dissertation.pdf)
