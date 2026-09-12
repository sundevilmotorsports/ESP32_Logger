# Data Logger Corruption Investigation Handoff

## Purpose and current state

This document records the evidence, software findings, attempted fix, validation, remaining risks, and recommended next work from the investigation of `9_11_3.benji2` and `9_11_3.csv`.

The firmware reported by the user as flashed for the run was repository commit:

```text
3e6c8ee47cf28b078a649c549d5e2d698a032b5b
```

The SD write recovery changes described below are currently **uncommitted** modifications to:

- `main/main.c`
- `main/sdcard.c`
- `main/sdcard.h`

The raw and converted samples are also untracked in this repository:

- `9_11_3.benji2`
- `9_11_3.csv`

Do not assume the current fix has been tested on the logger hardware. It has compiled successfully, but fault injection and on-target validation remain outstanding.

## Source data and processing suite

The files investigated were:

- Raw log: `C:\Users\Alexs\Documents\GitHub\ESP32_Logger\9_11_3.benji2`
- Converted CSV: `C:\Users\Alexs\Documents\GitHub\ESP32_Logger\9_11_3.csv`
- GUI entry point: `C:\Users\Alexs\Documents\GitHub\processing\SDM26\SDM26_gui_converter.py`
- Converter pipeline: `C:\Users\Alexs\Documents\GitHub\processing\SDM26\conversion_pipeline.py`
- Converter device definitions: `C:\Users\Alexs\Documents\GitHub\processing\SDM26\devices.py`

Hashes of the analyzed inputs:

```text
9_11_3.benji2 SHA-256:
6d92646c0760f393dcf6aec159bc7d6f86f596271abe7f89cf1be6c57f7ab418

9_11_3.csv SHA-256:
cd792657387b0bf55f97f9f176157dcbcad602a9abb84d9ee600e324ad6fec6d
```

## Confirmed facts from the raw log

The `.benji2` layout in this build is:

- Four-byte little-endian header length
- Stored header length: 1,918 bytes
- Absolute start of record data: byte 1,922
- Record size: `CH_COUNT == 213` bytes
- Payload size: 1,363,200 bytes
- Complete records: exactly 6,400
- Trailing partial bytes: zero

The header ends with `RATEB1,CH_COUNT,`. The converter intentionally reads one fewer header byte and then skips the trailing comma before parsing data.

No second copy of the complete header signature was found in the file. This rules out an embedded header caused by reopening and appending to the same filename for this particular sample.

### Corrupt extents

The observed corruption occupies two exact absolute file ranges:

| Range | Size | Record boundary relationship |
|---|---:|---|
| `0x28000` through `0x67FFF` | 256 KiB | Starts in sample 761 at record byte 38; ends in sample 1991 at record byte 191 |
| `0xAC000` through `0xEFFFF` | 272 KiB | Starts in sample 3299 at record byte 116; ends in sample 4607 at record byte 39 |

Fully valid records resume at sample 1992 and sample 4608. The file never loses its 213-byte record phase.

The recovery is therefore not converter resynchronization. The bad ranges retain their full byte lengths, and valid bytes continue at their original offsets after each range.

### Sector and block evidence

The corrupt ranges are aligned to 16-KiB file offsets. Inside them:

- At least 174 complete 512-byte sectors are entirely `0x00` or entirely `0xFF`.
- Longest observed all-zero run: 13,312 bytes, exactly 26 sectors.
- Other long runs are exact multiples of 512 bytes and begin/end at 512-byte boundaries.
- There are 131 nonconstant sectors whose 512 bytes exactly match the sector 16 KiB earlier.
- The abnormal boundaries do not align with the 213-byte record size.
- The abnormal boundaries do not align with the firmware's 4,047-byte application chunks (`19 * 213`).

This is strong evidence of sector/block-level content replacement, stale data, or misdirected data. It is inconsistent with one bad sensor channel and inconsistent with a normal non-record-aligned short append.

The firmware mount configuration contains `.allocation_unit_size = 16 * 1024` in `main/sdcard.c`. ESP-IDF uses that value when formatting a volume; because `format_if_mount_failed` is false, this does not prove the card used for the run had 16-KiB clusters. The matching alignment is still significant.

## Converter findings

### The converter did not create the corrupt intervals

The GUI only dispatches to `convert_benji2_inputs_to_outputs()` in `conversion_pipeline.py`.

An independent parser was written in memory using the same `devices.py` field definitions and conversions. It decoded every raw record and compared the resulting strings with the supplied CSV:

```text
CSV rows compared: 6400
Mismatched rows: 0
Mismatched cells: 0
```

The strange CSV values are deterministic conversions of bytes already present in the raw file.

### Converter framing is fragile but was correct for this file

`conversion_pipeline.py` subtracts one from the header length at line 113 and consumes one byte at line 131. This depends on the firmware header ending in a comma. If the header format changes, the converter could shift the entire data stream by one byte.

That did not happen here. The final header byte is the expected comma, the payload divides exactly into 213-byte records, and all 6,400 rows reproduce exactly.

### Duplicate-header filtering is fragile

`filter_duplicate_headers()` strips trailing digits from every header token and counts duplicate base names to infer byte widths. A legitimate channel name ending in a number, or two different channels that collapse to the same base name, can be merged incorrectly.

This did not cause the intermittent corruption in `9_11_3`, but the file format should eventually carry explicit field widths rather than deriving them from names.

### No corruption detection or resynchronization

The converter reads fields sequentially until EOF. It does not check:

- Record magic or sync words
- Record sequence numbers
- Per-record or per-block CRCs
- Timestamp plausibility
- Channel ranges
- Incomplete trailing-record errors beyond silently stopping

As a result, it faithfully converts corrupted bytes into plausible-looking numeric columns and cannot identify or skip a damaged region safely.

### Scaling and signedness make corruption look more extreme

`devices.py` maps steering as:

```python
0.084769 * (raw - 1430)
```

Therefore:

- Raw `0x0000` becomes approximately `-121.21967` degrees.
- Raw `0xFFFF` becomes approximately `5434.116745` degrees.

Damper and strain-gauge calibration functions similarly magnify zero/`0xFF` fill patterns.

The battery voltage channel is configured as signed at `devices.py:95-100`, even though the INA260 bus-voltage register is unsigned. This can produce misleading negative values for otherwise valid raw bit patterns. It affects one channel's interpretation and did not create the broad corrupt intervals.

Sample-rate inference issues in the processing suite can affect MoTeC output timing but do not change values in the CSV conversion path.

## Firmware issues ranked by relevance to this incident

### 1. SD/FatFs commit results were ignored

**Incident likelihood: high. Severity: critical.**

Before the attempted fix, `fast_log_buffer()` checked only the immediate `fwrite()` byte count. The stream used a 4-KiB stdio buffer, so a successful `fwrite()` could mean only that data was copied into RAM. The eventual `fflush()` result was ignored. `sdcard_sync()` ignored both `fflush()` and `fsync()` results.

This allowed a real VFS, FatFs, or SDMMC commit error to occur without changing logger control flow. Later successful writes could then explain good data resuming after a fixed-length damaged area.

The raw file cannot distinguish these cases:

- The lower layer reported an error that firmware ignored.
- The lower layer reported success despite incorrect media contents.
- The copy/extraction path returned incorrect sectors.

The attempted fix addresses the first case. It does not yet detect silent incorrect writes that return success.

### 2. Active log files can be downloaded while they are being written

**Incident likelihood: high if the sample was downloaded during logging; otherwise not applicable. Severity: critical.**

`main/server.c:190` opens any requested SD file with `fopen(path, "rb")` and reads it without taking `log_file_mutex`. The logger can have the same file open for append.

`sdkconfig` has:

```text
CONFIG_FATFS_FS_LOCK=0
# CONFIG_FATFS_IMMEDIATE_FSYNC is not set
```

ESP-IDF's FatFs configuration states that when file locking is disabled, the application must avoid illegal duplicate opens and operations on open objects. The current code does not prevent this.

If `9_11_3.benji2` came from the HTTP endpoint while the logger was still writing, the downloaded copy may be inconsistent. If it was copied directly from the card after the logger closed the file, this path cannot explain that copy.

### 3. GNSS parsers can overwrite their task stack

**Incident likelihood: low to medium for this sector-shaped trace. Severity: critical.**

`main/gnss.c:88` and `main/gnss.c:140` use unbounded `%[^,]` conversions with 16-byte destination arrays. A malformed or unexpectedly long NMEA field can overflow the arrays and corrupt the task stack.

This is a real arbitrary-memory-corruption path and must be fixed. Exact 512-byte and 16-KiB storage patterns make it a weaker explanation for this particular file.

### 4. Failed writes previously discarded staged records

**Incident likelihood for `9_11_3`: low. Severity: high.**

Before the attempted fix, `log_flush_task()` removed up to 256 records from the ring into `log_flush_staging`, then discarded all remaining staged data when `fast_log_buffer()` failed.

A partial write whose length was not divisible by 213 would shift every later record permanently. Normal later flushes cannot restore alignment because every normal write is an integer number of 213-byte records. The analyzed file remains aligned and therefore does not show this failure mode.

A zero-byte failure or a failure exactly divisible by 213 could preserve alignment, but it would create missing timestamps rather than fixed-width zero/`0xFF` and duplicated storage sectors.

### 5. Append mode can place another header inside an existing file

**Incident likelihood for `9_11_3`: ruled out. Severity: high.**

`open_log_file()` uses append mode and unconditionally writes a new header. Reusing a filename would append another four-byte header length and header to the old payload. The converter has no embedded-header detection or resynchronization.

No repeated header signature was found in this raw file, so this did not cause the examined corruption.

### 6. ADC SPI receive handling has correctness and memory-safety risks

**Incident likelihood for the cross-channel trace: low. Severity: high.**

`main/adc.c` uses `SPI_DMA_CH_AUTO` with a two-byte stack receive buffer. ESP-IDF DMA transfers have alignment and word-sized receive-buffer requirements that can make a two-byte DMA target unsafe.

All calls to `spi_device_transmit()` in `read_all_channels()` ignore their return values. `adc_read_sync()` then always returns `ESP_OK`, so failed transfers can publish stale or uninitialized ADC data.

These issues can corrupt analog measurements, including steering, but cannot explain timestamp and unrelated channels being replaced together at storage-sector boundaries.

### 7. CAN receive task is created twice

**Incident likelihood for the storage trace: low. Severity: medium to high.**

`can_init()` creates `can_receive_task()` at `main/can.c:104`. Immediately afterward, `tasks_start_all()` creates a second instance at `main/tasks.c:37`.

Two consumers read the same queue and call `process_can_message()` concurrently. This can reorder updates and creates races on shared channel structures. It does not explain the sector-aligned raw-file corruption.

The return value from `xQueueSendFromISR()` is also ignored, so a full CAN queue silently drops messages.

### 8. Shared channel structures are not captured atomically

**Incident likelihood for the storage trace: low. Severity: medium.**

CAN, GNSS, IMU, DTC, and logger tasks update/read shared structures without a single snapshot lock. One 213-byte record can contain fields from slightly different update moments or torn multi-byte values.

This can cause isolated inconsistent channels. It cannot account for whole 512-byte sectors filled with constants or repeated at 16-KiB offsets.

### 9. Ring flushing is triggered only after a rejected sample

**Incident likelihood for corruption: unrelated. Severity: medium for data completeness.**

`logBuffer_task()` notifies the flush task only when `log_ring_write()` fails because the 256-entry ring is already full. The sample that triggers the flush is dropped. This creates periodic timestamp gaps under normal operation and increments the ring overrun counter.

This is separate from byte corruption and should be redesigned using a high-water mark or periodic notification.

### 10. Tire-temperature CAN copies use the wrong direction

**Incident likelihood for corruption: unrelated. Severity: medium for those channels.**

Cases `0x371`, `0x372`, `0x381`, `0x382`, `0x391`, `0x392`, `0x3A1`, and `0x3A2` call forms such as:

```c
memcpy(data, &flt.tiretemp1, sizeof(flt.tiretemp1));
```

This copies the stored value into the received CAN buffer. The intended direction is presumably from `data` into the tire-temperature field. Those logged tire-temperature values therefore do not update correctly.

### 11. Lambda conversion performs integer division

**Incident likelihood for corruption: unrelated. Severity: low to medium.**

`engine.lambda1` is a `uint8_t`, and `data[1] / 100` performs integer division. Values below 100 become zero and the field cannot represent the intended fractional lambda value. The field type, raw representation, and converter scaling need to be made consistent.

### 12. Binary records have no intrinsic integrity metadata

**Incident likelihood: this did not create corruption, but it prevents reliable detection and recovery. Severity: high.**

Records contain raw channel bytes only. There is no magic value, format version, record length, sequence number, or checksum. Once bytes are overwritten, the converter cannot distinguish bad values from valid samples or prove where valid records resume.

## Attempted SD write recovery fix

The current working tree contains an implementation intended to stop detected write failures from silently losing staged records.

### Changes in `main/sdcard.c`

1. Added `log_committed_size`, which records the end offset of the last fully confirmed header or data chunk.
2. Added `sync_log_file_locked()`:
   - Checks `fflush()`.
   - Checks `fileno()`.
   - Checks `fsync()`.
   - Returns failure instead of silently continuing.
3. Added `rollback_log_file_locked()`:
   - Clears the stream error.
   - Uses `ftruncate()` to restore the file to `log_committed_size`.
   - Restores the stream position with `fseeko()`.
   - Calls and checks `fsync()` for the rollback.
4. Changed the active log stream to unbuffered mode with `_IONBF`. The flush task already groups records into approximately 4-KiB chunks. This makes `fwrite()` report the underlying VFS write result rather than merely accepting bytes into a stdio buffer.
5. The header is now synced successfully before its end offset becomes the committed size.
6. Closing an existing active file is refused if its final sync fails.
7. `fast_log_buffer()` now:
   - Confirms the current file has not shrunk below the committed size.
   - Removes any uncommitted tail left by a previous failure.
   - Writes at the committed end offset.
   - Requires the full `fwrite()` count.
   - Requires `fflush()` and `fsync()` success.
   - Verifies the resulting file size.
   - Advances `log_committed_size` only after all checks pass.
   - Rolls back to the prior committed size on any detected failure.
8. `sdcard_sync()` now returns `esp_err_t`, and its caller logs failures.

### Changes in `main/main.c`

`log_flush_task()` now keeps `pending_entries` and `pending_entry_index` across retries. If `fast_log_buffer()` fails:

- The failed chunk and all later staged records remain in `log_flush_staging`.
- The same chunk is retried after 100 ms.
- The failure is logged on the first attempt and every tenth attempt.
- The pending index advances only after a durable successful commit.
- Recovery is logged with the retry count.

While the 256-record staging buffer is retained, the normal 256-record ring can accept additional samples. At 100 Hz, the two buffers provide roughly 5.1 seconds of total RAM capacity from the start of a failure. A longer failure can still cause newly acquired samples to overrun once both buffers are full.

### What the attempted fix protects against

- A short `fwrite()` that returns fewer bytes than requested
- A reported `fflush()` failure
- A reported `fsync()`/FatFs `f_sync()` failure
- A file-size mismatch after commit
- Retrying a partially appended chunk without duplicating its prefix
- Discarding the remainder of an already drained staging batch

### What the attempted fix does not protect against

- A driver, controller, or medium returning success while storing incorrect bytes
- Corruption that appears only after later reads
- Power loss while a retry is pending in RAM
- Failures lasting longer than the finite staging plus ring capacity
- Concurrent HTTP reads of the active file
- Record-level detection or resynchronization in the converter
- A log-file rotation request while records for the previous file remain pending

The last point requires special attention: `log_flush_task()` releases `log_file_mutex` between chunks. Another task can switch the active file while a staging batch is pending. Remaining records could then be written into the new file. This risk existed before the fix and the persistent retry state makes it more important to resolve explicitly.

## Validation performed

The complete firmware was built with ESP-IDF 5.5.1 after the source changes:

```text
Project build complete.
Generated: build/logger.bin
logger.bin size: 0xF2F70 bytes
Smallest app partition: 0x100000 bytes
Free space: 0xD090 bytes (5%)
```

`main/main.c` and `main/sdcard.c` compiled and the final firmware linked successfully.

`git diff --check` passed. Git reported only the repository's LF-to-CRLF conversion warnings.

Build warnings observed but not introduced by this fix included:

- The project passes `-std=gnu++23` to C compilation units.
- `adc_data_mutex` is unused.
- `ModuleCore` has missing-field-initializer warnings for TWAI timing fields.
- `ESP_ROM_ELF_DIR` was missing for optional GDB-init generation; this was a CMake warning and did not stop the successful build.

No logger hardware was flashed or exercised. No forced `fwrite`, `fflush`, `fsync`, or rollback failure has been tested yet.

## Prioritized work for the next session

### P0: Validate and harden the attempted write fix

1. Add controllable fault injection around file operations. Exercise at least:
   - `fwrite()` returns zero.
   - `fwrite()` returns a non-record-aligned short count.
   - `fflush()` fails after a full `fwrite()`.
   - `fsync()` fails after a full logical append.
   - `ftruncate()` rollback fails once and later recovers.
   - The file is externally shortened or unexpectedly lengthened.
2. Verify the final byte stream contains every staged record exactly once and remains aligned to 213 bytes after each recovery.
3. Run an on-target soak test and measure how long each approximately 4-KiB `fsync()` takes. The current fix performs a durable sync for every chunk, which may increase latency and SD metadata traffic.
4. Decide whether to commit whole staging batches or larger groups per `fsync()` while preserving rollback and retry semantics.
5. Define behavior for a permanently failed or removed card. Options include remount/reopen recovery, a visible fault state, stopping acquisition, or spilling to another storage area.
6. Coordinate file rotation with the flush task so a new file cannot become active until all records assigned to the previous file are committed.

### P0: Remove active-file read/write overlap

1. Prevent the HTTP server from opening the current active log, or create a synchronized immutable snapshot after a successful sync.
2. Protect all SD file operations with an appropriate filesystem/file policy, not only the logger's writer mutex.
3. Evaluate enabling a nonzero `CONFIG_FATFS_FS_LOCK` and confirm the selected count covers every simultaneous open file.
4. Record whether future diagnostic samples were copied from a stopped card or downloaded while logging.

### P0: Fix memory-safety defects

1. Add field widths to all GNSS `sscanf()` scans, or replace them with bounded token parsing.
2. Allocate an SPI DMA-capable, correctly aligned receive buffer of the required rounded size for ADC transactions, or disable DMA for these two-byte transfers.
3. Check every `spi_device_transmit()` result and propagate failure through `adc_read_sync()`.

### P1: Add log integrity and recovery metadata

1. Version the `.benji2` format.
2. Add a per-record magic value, monotonically increasing sequence number, explicit record length, and CRC.
3. Consider block-level commit markers or CRCs matching the logger's write batches.
4. Update the converter to report corrupt ranges, reject invalid records, and resynchronize only on validated magic/length/CRC combinations.
5. Preserve compatibility with existing header-only files or provide an explicit legacy converter path.

### P1: Correct acquisition and scheduling defects

1. Remove one of the two `can_receive_task()` creation sites.
2. Check `xQueueSendFromISR()` and count/report CAN queue drops.
3. Protect or atomically snapshot multi-byte shared channel data before building each log record.
4. Notify the flush task at a high-water mark or on a timer instead of waiting for a rejected ring write.
5. Preserve and expose ring-overrun counts in diagnostics.

### P1: Correct channel-specific data bugs

1. Reverse the tire-temperature `memcpy()` arguments so received CAN bytes update the stored values.
2. Redesign lambda storage/conversion to avoid integer division and preserve the intended resolution.
3. Correct `v_batt` signedness in `devices.py` after confirming the firmware's raw INA260 representation.
4. Audit all converter signedness, endianness, calibration factors, and units against the firmware and sensor datasheets.

### P2: Make the header self-describing and robust

1. Store explicit channel names, byte widths, signedness, units, and scale metadata.
2. Remove the converter's `header_length - 1` and one-byte skip convention.
3. Stop deriving field widths by stripping digits from channel names.
4. Reject or explicitly handle an embedded header.
5. Avoid append mode for a newly numbered log unless continuation is explicitly requested and validated.

## Suggested acceptance criteria

Before treating the corruption issue as resolved, demonstrate all of the following:

1. A forced non-record-aligned short write recovers with no missing, duplicated, or shifted records.
2. A forced `fsync()` failure retries the same chunk and preserves the last committed file boundary.
3. A rollback failure does not append new data beyond the uncertain tail.
4. The logger clearly reports a sustained storage fault and its overrun count.
5. File rotation cannot split one staging batch across two log files.
6. The active log cannot be downloaded concurrently with writes.
7. A multi-hour on-target log has monotonic sequence numbers, valid CRCs, and no unexpected zero/`0xFF` or duplicated sectors.
8. The converter identifies deliberately corrupted records and resumes only at a validated record boundary.

