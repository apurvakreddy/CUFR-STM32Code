import sys
import pandas as pd

# ============================================================
# INTERVIEW FOCUS: OFFLINE CONFIGURATION -> settings.bin
# This script converts the human-edited CSV packet settings
# into a compact binary file that the STM32 can read at boot.
# ============================================================

# Run as:
# python create_bin.py <path_to_csv>

def main(args):
    # Expect exactly one argument: the path to the CSV settings file.
    if len(args) != 1:
        print("No path provided.")
        return

    csv_file = None
    try:
        # Load the packet-definition spreadsheet exported as CSV.
        csv_file = pd.read_csv(args[0])
    except:
        print("failed to open file")
        return

    # Open output file in binary write mode.
    # This is the file copied onto the SD card for the car-side MCU.
    f = open("settings.bin", "wb")

    # ------------------------------------------------------------
    # Header byte 0: total number of individual CAN messages
    # that will be described in the file.
    # ------------------------------------------------------------
    msg_count = int(csv_file.iat[0, 0])
    t = msg_count.to_bytes()
    f.write(t)

    # ------------------------------------------------------------
    # Next 3 records: packet-type metadata for stat / medium / fast
    # Each packet entry writes:
    #   id            -> packet identifier character
    #   length        -> packet payload length
    #   speed         -> packet transmit period, scaled by /10
    #   num_messages  -> number of CAN messages mapped into packet
    # ------------------------------------------------------------
    for i in range(3):
        id = csv_file.iat[i+1, 0].encode('utf-8')         # Packet ID: e.g. 's', 'm', 'f'
        length = int(csv_file.iat[i+1, 1]).to_bytes()     # Packet payload length in bytes
        speed = int(csv_file.iat[i+1, 2]/10).to_bytes()   # Store speed in compact scaled form
        num_messages = int(csv_file.iat[i+1, 3]).to_bytes()  # Number of CAN signals in this packet

        f.write(id)
        f.write(length)
        f.write(speed)
        f.write(num_messages)

    # ------------------------------------------------------------
    # Packet payload offsets and per-packet indicator-bit indexes.
    # Start at byte index 3 because:
    #   byte 0 = packet ID
    #   byte 1 = indicator bits
    #   byte 2 = indicator bits
    # Payload data starts after those first 3 bytes.
    # ------------------------------------------------------------
    med_idx = 3
    med_i = 0
    fast_idx = 3
    fast_i = 0
    stat_idx = 3
    stat_i = 0

    # ------------------------------------------------------------
    # Write one 10-byte record for each CAN message.
    # Each record tells the MCU:
    #   - which CAN ID to match
    #   - which packet it belongs to
    #   - where in the packet payload to place it
    #   - how many bytes of CAN data are relevant
    #   - which bytes to send
    #   - how many bytes are sent over radio
    #   - which indicator-bit position belongs to this message
    # ------------------------------------------------------------
    for i in range(msg_count):
        # CAN message ID from CSV, stored as 2 bytes.
        msg_id = int(csv_file.iat[i+4, 0], 16).to_bytes(2)
        f.write(msg_id)

        # Packet assignment: 's', 'm', or 'f'
        pkt = csv_file.iat[i+4, 4]
        f.write(pkt.encode('utf-8'))

        # Defaults before packet-specific placement is calculated.
        start = 0
        idx = 0

        # Number of payload bytes this message occupies in the radio packet.
        len_to_write = int(int(csv_file.iat[i+4, 3]) / 8)

        # Place the message into the correct packet and assign:
        #   start = byte offset in that packet
        #   idx   = indicator-bit index within that packet
        if pkt == "s":
            start = stat_idx
            stat_idx += len_to_write
            idx = stat_i
            stat_i += 1
        elif pkt == "m":
            start = med_idx
            med_idx += len_to_write
            idx = med_i
            med_i += 1
        elif pkt == "f":
            start = fast_idx
            fast_idx += len_to_write
            idx = fast_i
            fast_i += 1

        # Start byte offset of this message inside its packet.
        f.write(start.to_bytes())

        # Number of useful data bytes from the CAN message.
        data_len = int(int(csv_file.iat[i+4, 1]) / 8)
        f.write(data_len.to_bytes())

        # Bitmask of which CAN bytes should be transmitted.
        # Currently hardcoded to 0xFFFFFF, meaning "send all relevant bytes".
        # The commented code below suggests this could be made more selective.
        bytes_to_send = str("0xFFFFFF")
        # if data_len != len_to_write:
        #     bytes_to_send = csv_file.iat[i+4, 2]
        b_t_s = int(bytes_to_send, 16)
        f.write(b_t_s.to_bytes(3))

        # Number of bytes to place in the outgoing radio packet.
        f.write(len_to_write.to_bytes())

        # Per-packet message index used for indicator-bit tracking.
        f.write(idx.to_bytes())

    # Finished building binary configuration file.
    f.close()

if __name__ == "__main__":
    main(sys.argv[1:])
