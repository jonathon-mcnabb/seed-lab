import smtplib
import socket
import fcntl
import struct

def get_ip_address(ifname):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return socket.inet_ntoa(fcntl.ioctl(
        s.fileno(),
        0x8915,  # SIOCGIFADDR
        struct.pack('256s', ifname[:15])
    )[20:24])

print get_ip_address('lo')
host_ip = get_ip_address('eth0')
    # Driver code

TO = 'EMAIL_SENDING_TO'
SUBJECT = 'SSH Command'
TEXT = 'Please run the following command to ssh into the raspberry pi: \n\n' \
       'ssh pi@' + str(host_ip)

# Gmail Sign In
gmail_sender = 'EMAIL_USERNAME'
gmail_passwd = 'EMAIL_PASSWORD!'

server = smtplib.SMTP('smtp.gmail.com', 587)
server.ehlo()
server.starttls()
server.login(gmail_sender, gmail_passwd)

BODY = '\r\n'.join(['To: %s' % TO,
                    'From: %s' % gmail_sender,
                    'Subject: %s' % SUBJECT,
                    '', TEXT])

try:
    server.sendmail(gmail_sender, TO, BODY)
    print('email sent')
except:
    print('error sending mail')

server.quit()
