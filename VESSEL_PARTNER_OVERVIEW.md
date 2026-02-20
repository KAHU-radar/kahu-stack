# KAHU Marine Radar Intelligence — Vessel Partner Overview

*For vessel owners and operators considering a KAHU pilot installation*

---

## What Is KAHU?

KAHU is a lightweight software system that runs on a small computer (Raspberry Pi) aboard your vessel. It connects to your existing Navico or Halo marine radar and adds intelligent target tracking — automatically detecting, labelling, and logging nearby vessels, buoys, and obstacles.

The data is sent to a secure cloud dashboard accessible from anywhere, so you and your team can review historical traffic, replay encounters, and analyse patterns long after a voyage is complete.

---

## What We're Asking For

We'd like to **test KAHU aboard your vessel** during normal operations.

- **No changes to your radar** — we tap in passively over the existing ethernet connection
- **No changes to your navigation software** — we run alongside everything you already use
- **One small device** — a Raspberry Pi about the size of a deck of cards, tucked out of the way
- **Your time: ~30 minutes** for initial setup, then nothing — it runs automatically

---

## How It Works (Plain Language)

```
Your Navico/Halo Radar
        │  (existing ethernet cable — no modification)
        ▼
Raspberry Pi  ←─── GPS antenna or NMEA feed
  (on your vessel)
        │  (cellular or wifi when in range)
        ▼
KAHU Cloud Dashboard
  (accessible from any browser)
```

1. Your radar sweeps as normal. KAHU listens on the same network — it does not send any commands to the radar.
2. The Pi identifies radar returns, clusters them into targets, and records their positions relative to your vessel.
3. Every few seconds, that data is uploaded to the KAHU cloud backend.
4. You (and our team) can view a live or historical map of all tracked targets at `kahu.earth`.

---

## What We Measure

| Data collected | Purpose |
|---|---|
| Radar target positions (range + bearing from your vessel) | Core tracking — where are other vessels? |
| Your vessel GPS position and heading | Convert relative targets to absolute lat/lon |
| Timestamp of each detection | Replay, analysis, incident review |

**We do not collect:** AIS transponder IDs, vessel names, your navigation plans, chart data, engine data, or anything from your chartplotter or VHF.

---

## Privacy & Data Ownership

- All data collected belongs to you. We store it under a vessel ID you choose.
- Data is encrypted in transit (HTTPS) and at rest (PostgreSQL on Google Cloud).
- You can request deletion of all your data at any time.
- We will not share your data with third parties without your written consent.
- If the Pi has no internet connection, it stores data locally and uploads when connectivity returns. Nothing is lost.

---

## What You Get

- **Live radar target dashboard** — view tracked contacts in a browser during or after a voyage
- **Historical replay** — review any past session
- **Full access to your own data** — export as CSV/JSON on request
- **Zero ongoing cost** during the pilot — we cover all cloud hosting

---

## Risk Assessment

| Concern | Reality |
|---|---|
| Will it affect my radar? | No. The Pi only listens on the network — it cannot send commands to the radar hardware. |
| Will it slow my radar down? | No. Radar speaks to chartplotter over multicast UDP. We join that same multicast group passively. |
| What if the Pi crashes? | It reboots itself automatically. Your radar and navigation are completely unaffected either way. |
| What if I want it removed? | Unplug the Pi. Done. Nothing installed on the radar or chartplotter. |
| Does it need internet? | No — it buffers data locally if offline and uploads when connection is available. |
| What does it cost to run? | Nothing during the pilot. Power draw is ~5 W (less than a phone charger). |

---

## We've Already Tested This

Before approaching vessel partners, we validated the full system end-to-end in simulation:

- A **radar emulator** replicates the exact multicast UDP protocol used by Navico/Halo hardware
- The emulator generates realistic vessel traffic in San Diego waters with proper ARPA target data
- Our perception stack (EchoFlow Lite) runs on the Pi and processes this data identically to how it will process real radar returns
- The cloud pipeline, dashboard, and offline buffering have all been exercised

**The only difference between our tested simulation and your real installation is the source of the radar data.** Mayara — the open-source Rust library we use to connect to the radar — handles that translation and auto-discovers your radar on the network with no configuration required.

Below is a screenshot of the Foxglove visualisation from our simulation, showing tracked targets overlaid on a map:

*(screenshot to be inserted — contact us for a live demo)*

---

## The Hardware

| Item | Details |
|---|---|
| Raspberry Pi 4 or 5 | 4 GB RAM minimum |
| MicroSD card | 32 GB+ (Class 10) |
| Power supply | 5V/3A USB-C (included) |
| Short ethernet cable | Connects to your radar network switch |

Total hardware cost: ~$80–120. We provide this during the pilot.

---

## Installation Process

1. We arrive with the Pi pre-configured for your vessel
2. We connect it to your radar network (one ethernet cable to your existing switch)
3. We connect a GPS source (your NMEA talker, or a small USB GPS dongle — $15)
4. We power it on and confirm it's seeing radar targets — typically takes under 5 minutes
5. You get login credentials for your dashboard

That's it. The system is designed to be installed by a non-engineer and require zero maintenance.

---

## Questions?

| Question | Contact |
|---|---|
| Technical questions about the system | [your-email@domain.com] |
| Data privacy questions | [your-email@domain.com] |
| To schedule an installation | [your-email@domain.com] |

---

*KAHU uses open-source components including [Mayara](https://github.com/MarineYoughtRadar/mayara-server) (radar interface), [ROS2](https://ros.org) (robotics middleware), and [EchoFlow Lite](https://github.com/KAHU-radar/echoflow-lite) (target detection). Source code available on request.*
