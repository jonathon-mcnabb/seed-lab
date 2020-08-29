import smtplib
import os

#'\' is used to splite pythone line
ipaddress = os.popen("hostname -I").read()
ssid = os.popen("iwconfig wlan0 \
                | grep 'ESSID' \
                | awk '{print $4}' \
                | awk -F\\\" '{print $2}'").read()

print("ssid: " + ssid)
print("ipaddress: " + ipaddress)
host_ip = ipaddress
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
