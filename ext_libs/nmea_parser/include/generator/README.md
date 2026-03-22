- [NMEA Generator — Developer Guide](#nmea-generator--developer-guide)
- [Coverage Matrix — Primary NMEA Sentences vs. Shared Buffers](#coverage-matrix--primary-nmea-sentences-vs-shared-buffers)
  - [Data sources](#data-sources)
  - [Sentence-by-sentence matrix](#sentence-by-sentence-matrix)
    - [GGA — Global Positioning System Fix Data](#gga--global-positioning-system-fix-data)
    - [RMC — Recommended Minimum Specific GNSS Data](#rmc--recommended-minimum-specific-gnss-data)
    - [GSA — GNSS DOP and Active Satellites](#gsa--gnss-dop-and-active-satellites)
    - [GSV — GNSS Satellites in View](#gsv--gnss-satellites-in-view)
    - [VTG — Course Over Ground and Ground Speed](#vtg--course-over-ground-and-ground-speed)
    - [GLL — Geographic Position — Latitude/Longitude](#gll--geographic-position--latitudelongitude)
    - [ZDA — Time and Date](#zda--time-and-date)
  - [Summary](#summary)

## NMEA Generator — Developer Guide

**Reference**: https://github.com/nguyenchiemminhvu/nmea_parser/tree/main/include/generator

## Coverage Matrix — Primary NMEA Sentences vs. Shared Buffers

### Data sources
- `location_data` — UBX-NAV-PVT (HP commit)
- `satellites_data` — UBX-NAV-SAT (LP commit)
- `timing_data` — UBX-TIM-TP, UBX-NAV-TIMEUTC, UBX-NAV-TIMEGPS, UBX-NAV-CLOCK
- `attitude_data` — UBX-NAV-ATT
- `dop_data` — UBX-NAV-DOP

### Sentence-by-sentence matrix

#### GGA — Global Positioning System Fix Data

| NMEA Field | Required | Source | Notes |
|---|---|---|---|
| UTC time | ✓ | `location_data.{hour,minute,second,millisecond}` | Direct |
| Latitude | ✓ | `location_data.latitude_deg` | Direct (decimal degrees) |
| Longitude | ✓ | `location_data.longitude_deg` | Direct |
| Fix quality | ✓ | `location_data.fix_type` + `flags2` | Derived: fix_type→quality, carrSoln→RTK |
| Num satellites | ✓ | `location_data.num_sv` | Direct |
| HDOP | ✓ | `dop_data.hdop` | Direct |
| Altitude MSL | ✓ | `location_data.altitude_msl_mm / 1000.0` | Unit conversion mm→m |
| Geoid separation | ✓ | `(altitude_ellipsoid_mm - altitude_msl_mm) / 1000.0` | Approximation; exact requires EGM96 lookup |
| DGPS age | optional | Not available | Left empty (valid per NMEA spec) |
| DGPS station ID | optional | Not available | Left empty |

---

#### RMC — Recommended Minimum Specific GNSS Data

| NMEA Field | Required | Source | Notes |
|---|---|---|---|
| UTC time | ✓ | `location_data.{hour,minute,second,millisecond}` | Direct |
| Status (A/V) | ✓ | `location_data.flags` bit 0 (gnssFixOK) + `fix_type >= 2` | Derived |
| Latitude | ✓ | `location_data.latitude_deg` | Direct |
| Longitude | ✓ | `location_data.longitude_deg` | Direct |
| Speed (knots) | ✓ | `location_data.ground_speed_mmps / 514.444` | Unit conversion |
| Course true | ✓ | `location_data.motion_heading_deg` | Direct |
| Date | ✓ | `location_data.{day,month,year}` | Direct |
| Magnetic variation | optional | `location_data.magnetic_declination_deg` | Emitted when |dec| > 0.0001° |
| Mode indicator | ✓ | Derived from `fix_type` + `flags` | A/D/E/N |

**Coverage: FULL**

---

#### GSA — GNSS DOP and Active Satellites

| NMEA Field | Required | Source | Notes |
|---|---|---|---|
| Op mode (A/M) | ✓ | Hardcoded 'A' (u-blox auto mode) | |
| Nav mode (1/2/3) | ✓ | `location_data.fix_type` | 2D/3D mapped directly |
| Satellite IDs (12 slots) | ✓ | `satellites_data.svs[].{sv_id, used}` | Filtered by `used==true` |
| PDOP | ✓ | `dop_data.pdop` | Direct |
| HDOP | ✓ | `dop_data.hdop` | Direct |
| VDOP | ✓ | `dop_data.vdop` | Direct |
| System ID | optional (NMEA 4.11) | Left as 0 (omitted) | Multi-constellation → use GNGSA |

---

#### GSV — GNSS Satellites in View

| NMEA Field | Required | Source | Notes |
|---|---|---|---|
| Total msg count | ✓ | Computed from satellite count | ceil(n/4) |
| Message number | ✓ | Loop counter | |
| Total SVs in view | ✓ | `satellites_data.num_svs` (filtered by constellation) | |
| SV ID | ✓ | `satellites_data.svs[].sv_id` | |
| Elevation | ✓ | `satellites_data.svs[].elevation_deg` | |
| Azimuth | ✓ | `satellites_data.svs[].azimuth_deg` | |
| SNR | ✓ | `satellites_data.svs[].cno` | |
| Talker per constellation | ✓ | `satellites_data.svs[].gnss_id` | GP/GL/GA/GB/GQ mapped |

**Coverage: FULL**

---

#### VTG — Course Over Ground and Ground Speed

| NMEA Field | Required | Source | Notes |
|---|---|---|---|
| Course true | ✓ | `location_data.motion_heading_deg` | |
| Course magnetic | optional | `motion_heading_deg + magnetic_declination_deg` | Omitted when |dec| ≈ 0 |
| Speed knots | ✓ | `ground_speed_mmps / 514.444` | |
| Speed km/h | ✓ | `ground_speed_mmps / 1000.0 * 3.6` | |
| Mode | ✓ | Derived from `fix_type` + `flags` | |

**Coverage: FULL**

---

#### GLL — Geographic Position — Latitude/Longitude

| NMEA Field | Required | Source | Notes |
|---|---|---|---|
| Latitude | ✓ | `location_data.latitude_deg` | |
| Longitude | ✓ | `location_data.longitude_deg` | |
| UTC time | ✓ | `location_data.{hour,minute,second,millisecond}` | |
| Status | ✓ | `flags` bit 0 + `fix_type` | |
| Mode | ✓ | Derived | |

**Coverage: FULL**

---

#### ZDA — Time and Date

| NMEA Field | Required | Source | Notes |
|---|---|---|---|
| UTC time | ✓ | `location_data` or `timing_data.utc_*` fields | Prefer timing_data for sub-second accuracy |
| Day / month / year | ✓ | Same sources | |
| Timezone offset | optional | Hardcoded 0,0 (UTC) | GNSS always reports UTC |

**Coverage: FULL**

---

### Summary

| Sentence | Coverage | Confidence | Missing |
|---|---|---|---|
| GGA | Full | ✓ High | — |
| RMC | Full | ✓ Full | — |
| GSA | Full | ✓ Full | — |
| GSV | Full | ✓ Full | — |
| VTG | Full | ✓ Full | — |
| GLL | Full | ✓ Full | — |
| ZDA | Full | ✓ Full | — |

**Conclusion:** A full primary NMEA epoch can be generated.