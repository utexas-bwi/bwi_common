import hashlib, os, time, urllib2
from subprocess import check_output

HASH_ITERATIONS = 1000
SECRETKEY_LOCATION = "/home/bwilab/.cmasskey"
BASE_URL = "http://nixons-head.csres.utexas.edu:7978/update?"

hostname = check_output("hostname").rstrip()
username = os.getlogin().rstrip()

def cmass_hash(msg):
    salt = open(SECRETKEY_LOCATION, "r").read().rstrip()
    for i in range(HASH_ITERATIONS):
        msg = hashlib.sha256(msg+salt).hexdigest()
    return msg

ip_address = check_output("ip addr | grep 'state UP' -A2 | tail -n1 | awk '{print $2}' | cut -f1 -d'/'", shell=True)
# based on updater from Pato's smallDNS (TODO: fix? It works reliably on all robots though)
try:
    old_ip_address = open(".oldIP", "r").read().rstrip()
except IOError:
    old_ip_address = "" # so that it will enter the conditional and create .oldIP

if ip_address != old_ip_address: #the IP has changed since the last execution

    ### make a request to the server updating the IP and active user
    timestamp = int(time.time())
    params = "name=" + hostname + "&timestamp=" + str(timestamp) + "&user=" + username
    token = cmass_hash(params)
    url = BASE_URL + params + "&token=" + token
    urllib2.urlopen(url).read()

    ### update the stored IP so we can detect the next change
    with open(".oldIP", "w+") as f:
        f.write(ip_address)
