import socket
import csv
import msvcrt
import time

UDP_IP = "0.0.0.0"
UDP_PORT = 4322

while True:
    print("Rozpoczynanie pomiaru.")
    # csv_filename = 'Data_' + time.strftime("%Y%m%d-%H%M%S") + '.csv'
    csv_filename = 'Data.csv'
    
    f = open(csv_filename, "w")
    f.truncate()
    f.close()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))

    with open(csv_filename, 'w', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)

        try:
            while True:
                data, addr = sock.recvfrom(1024)

                message = data.decode('utf-8')
                print("Odebrano pakiet: ", message)

                if message.endswith('Z'):
                    print("Zakończono pomiar. Wcisnij dowolny przycisk by rozpoczac kolejny.")
                    while not msvcrt.kbhit():
                        pass
                    msvcrt.getch()
                    break

                if message.startswith('DC_Remote_Car_Key'):
                    message = message[17:]
                    index_a = message.find('A')
                    index_b = message.find('B')
                    index_c = message.find('C')
                    index_d = message.find('D')
                    index_e = message.find('E')
                    index_f = message.find('F')
                    index_g = message.find('G')
                    index_h = message.find('H')
                    index_i = message.find('I')

                    value_a = message[index_a + 1:index_b]
                    value_b = message[index_b + 1:index_c]
                    value_c = message[index_c + 1:index_d]
                    value_d = message[index_d + 1:index_e]
                    value_e = message[index_e + 1:index_f]
                    value_f = message[index_f + 1:index_g]
                    value_g = message[index_g + 1:index_h]
                    value_h = message[index_h + 1:index_i]
                    value_i = message[index_i + 1:]

                    csv_writer.writerow([value_a, value_b, value_c, value_d, value_e, value_f, 
                                         value_g, value_h, value_i])
                    csvfile.flush()

        except Exception as e:
            print(f"Wystąpił błąd: {e}")
        finally:
            sock.close()
