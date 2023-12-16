import socket
import csv

UDP_IP = "127.0.0.1"
UDP_PORT = 4321

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

csv_filename = 'dane.csv'
with open(csv_filename, 'w', newline='') as csvfile:
    csv_writer = csv.writer(csvfile)

    try:
        while True:
            data, addr = sock.recvfrom(1024)

            message = data.decode('utf-8')

            if message.startswith('DC_Remote_Car_Key'):
                # Wyszukanie wartości po literze W i D
                index_w = message.find('W')
                index_d = message.find('D')

                if index_w != -1 and index_d != -1:
                    value_w = message[index_w + 1:index_d]
                    value_d = message[index_d + 1:message.find('E')]

                    # Zapisanie wartości do pliku CSV
                    csv_writer.writerow([value_w, value_d])
    except KeyboardInterrupt:
        print("Zatrzymano odbieranie pakietów UDP.")
        sock.close()
