import pynmea2

with open(r'/home/adit/Documents/DETRAS/100sample__buku.txt', mode='r') as in_file, open(r'/home/adit/Documents/DETRAS/100sample__buku.txt', mode='w') as out_file:

    for line in in_file:
        try:
            msg = pynmea2.parse(line)

            if msg.sentence_type == "RMC":

                gps_info_array = []
                gps_info_array.append(str(msg.datestamp))
                gps_info_array.append(str(msg.latitude))
                gps_info_array.append(str(msg.longitude))
                # gps_info_array.append(str(msg.))
                out_file.write(str(gps_info_array))
                out_file.write("\n")
        except pynmea2.ParseError as e:
            continue