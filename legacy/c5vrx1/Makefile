# SPDX-License-Identifier: GPL-3.0-only

.PHONY: test

test:
	$(CC) -std=c11 -Wall -Wextra -Werror -pedantic -Imain \
		main/c5vrx_stream.c tests/stream_test.c -o /tmp/c5vrx-stream-test
	/tmp/c5vrx-stream-test
	$(CC) -std=c11 -Wall -Wextra -Werror -pedantic -Imain \
		main/c5vrx_capabilities.c tests/capabilities_test.c -o /tmp/c5vrx-capabilities-test
	/tmp/c5vrx-capabilities-test
	$(CC) -std=c11 -Wall -Wextra -Werror -pedantic -Imain \
		main/c5vrx_cvbs_sync.c tests/cvbs_sync_test.c -o /tmp/c5vrx-cvbs-sync-test
	/tmp/c5vrx-cvbs-sync-test
	$(CC) -std=c11 -Wall -Wextra -Werror -pedantic -Imain \
		main/c5vrx_cvbs_levels.c tests/cvbs_levels_test.c -o /tmp/c5vrx-cvbs-levels-test
	/tmp/c5vrx-cvbs-levels-test
	$(CC) -std=c11 -Wall -Wextra -Werror -pedantic -Imain \
		main/c5vrx_av_clock.c tests/av_clock_test.c -o /tmp/c5vrx-av-clock-test
	/tmp/c5vrx-av-clock-test
	$(CC) -std=c11 -Wall -Wextra -Werror -pedantic -Imain \
		main/c5vrx_ring_tracker.c tests/ring_tracker_test.c -o /tmp/c5vrx-ring-tracker-test
	/tmp/c5vrx-ring-tracker-test
	python3 tools/c5vrx_usb_protocol.py --self-test
	python3 tools/c5vrx_lab.py self-test
	python3 tools/check_safe_usb_preview.py
	python3 tools/check_always_on_av.py
	python3 tools/validate_firmware_profiles.py
	@if [ -n "$(IDF_PATH)" ]; then python3 tools/audit_rf_dump_producer.py --idf "$(IDF_PATH)"; else echo "SKIP producer audit: IDF_PATH unset"; fi
