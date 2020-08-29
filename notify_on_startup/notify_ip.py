import smtplib
import socket

try:
    host_name = socket.gethostname()
    host_ip = socket.gethostbyname(host_name)
    print("IP : ", host_ip)
except:
    print("Unable to get Hostname and IP")

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
