import socket
import csv
import sys
import msvcrt

UDP_IP = "127.0.0.1"
UDP_PORT = 4321

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

csv_filename = 'Data.csv'
with open(csv_filename, 'w', newline='') as csvfile:
    csv_writer = csv.writer(csvfile)

    try:
        print("Program rozpoczął działanie. Oczekiwanie na pakiety UDP...")
        while True:
            data, addr = sock.recvfrom(1024)

            message = data.decode('utf-8')
            if message.startswith('DC_Remote_Car_Key'):
                index_l = message.find('L')
                index_p = message.find('P')

                if index_l != -1 and index_p != -1:
                    value_l = message[index_l + 1:index_p]
                    if message.endswith('E'):
                        value_p = message[index_p + 1:message.find('E')]
                    else:
                        value_p = message[index_p + 1:]

                    csv_writer.writerow([value_l, value_p])

            if message.endswith('E'):
                print("Program zakończył działanie.")
                sys.exit(0)
    except Exception as e:
        print(f"Wystąpił błąd: {e}")
    finally:
        sock.close()
