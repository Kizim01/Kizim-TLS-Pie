# MicroView OLED Status Reference

## Normal states
- READY: system idle and ready for a scan command.
- WAIT PI: MicroView sent record start and is waiting for Pi recording acknowledgement.
- SCANNING: active scan motion in progress.
- REC: Pi confirmed recording started.

## Error and recovery states
- PI TIMEOUT: Pi did not acknowledge recording start in time.
- REC LOST: recording dropped during scan; shown briefly before abort reason.
- PI ABORTED + CODE: Pi reported an abort reason.

## Pi abort reason codes
- 1: START TIMEOUT
- 2: NO INTERFACE
- 3: LIDAR OFFLINE
- 4: TCPDUMP ERROR
- 5: EMPTY PCAP
- 6: INTERRUPTED
- 7: TOOL MISSING

## Required status-return wiring
- Pi GPIO22 -> level shifter -> MicroView D4 (PISTATUS)
- Shared ground between Pi and MicroView is required.
