import paho.mqtt.client as mqtt
from prometheus_client import start_http_server, Summary, Gauge
import time
import datetime
import sqlite3
import os
from dotenv import load_dotenv
from sqlite3 import Error
from phue import Bridge
load_dotenv()

############ modify this for your prometheus variables #############
# There are different types of variables accepted by prometheus,
# since these are all point measurements in time, Gauge is the right choice

temp = Gauge('temperature', 'Internal Room Temperature')
hum = Gauge('humidity', 'Internal Room Humidity')
pres = Gauge('pressure', 'Internal Room Pressure')
dist = Gauge('distance', 'Internal Room Distance')
alt = Gauge('altitude', 'Internal Room Altitude')
light = Gauge('light', 'Internal Room Light')
#####################################################################

#####################################################################
# Specify Hue IP 
HueIP = os.environ.get("HUE_IP")

############ modify this for your mqtt config ##############
MQTT_ADDRESS = os.environ.get("MQTT_ADDRESS") 
MQTT_USER = os.environ.get("MQTT_USER") 
MQTT_PASSWORD = os.environ.get("MQTT_PASSWORD") 
MQTT_REGEX = 'home/([^/]+)/([^/]+)'
MQTT_CLIENT_ID = 'Zeus'
########################################################

def on_connect(client, userdata, flags, rc):
    """ Run the following when a client connects"""
    # There are various mqtt connect codes, only needed if debugging
    print('Connected with result code ' + str(rc)) 
    # Subscribe mqtt to the following variables
    client.subscribe([('indoor/conditions/temperature',1),('indoor/conditions/humidity',1),
                      ('indoor/conditions/altitude',1),
                      ('indoor/conditions/light',1),('indoor/conditions/pressure',1),
                      ('indoor/conditions/distance',1)])

def process_request(msg):
    """A function to read the published data over mqtt."""
    timeVal = datetime.datetime.now()
    # Print the timestep to make sure it is working now 
    print("Current Time:",datetime.datetime.now())
    # Print the message 
    msgStr = str(msg.payload)
    goodMsg = msgStr[2:-1]

    print(msg.topic + ' ' + goodMsg)


    # Make sure we associate prometheus logs with the correct mqtt variable
    # This publishes the mqtt variables to a prometheus gauge  
    # Also insert the data into the SQLite table 
    if msg.topic ==  'indoor/conditions/temperature':
        temp.set(msg.payload)
        sqlMsg = (str(timeVal),str(goodMsg),None,None,None);
        insert_database(sqlMsg)
    elif msg.topic == 'indoor/conditions/humidity':
        hum.set(msg.payload)
        sqlMsg = (str(timeVal),None,str(goodMsg),None,None);
        insert_database(sqlMsg)
    elif msg.topic == 'indoor/conditions/altitude':
        alt.set(msg.payload)
        sqlMsg = (str(timeVal),None,str(goodMsg),None,None);
        insert_database(sqlMsg)
    elif msg.topic == 'indoor/conditions/pressure':
        pres.set(msg.payload)
        sqlMsg = (str(timeVal),None,None,str(goodMsg),None);
        insert_database(sqlMsg)
    elif msg.topic == 'indoor/conditions/distance':
        dist.set(msg.payload)
    elif msg.topic == 'indoor/conditions/light':
        light.set(msg.payload)
        lightLum = goodMsg
        send_hueUpdate(lightLum)
    else:
        print('Incorrect topic')

def on_message(client, userdata, msg):
    """ Run the following command when a MQTT message is received""" 
    process_request(msg)
    
def send_hueUpdate(lumVal):
    """Update hue lights according to light value and time of day"""

  

    # Set up Hue bridge connection
    b = Bridge(HueIP)
    b.get_api()

    # Check if luminosity and time meet conditions to turn on
    # I could set a schedule, leaving it this way for now
    if (float(lumVal) < 10):
        # Specify times of day to limit hue updates
        now = datetime.datetime.now()
        todayLateLim = now.replace(hour=23, minute=45, second=0, microsecond=0)
        todayLate = now.replace(hour=20, minute=30, second=0, microsecond=0)
        todayEarly = now.replace(hour=6, minute=30, second=0, microsecond=0)
        todayAft = now.replace(hour=17, minute=0, second=0, microsecond=0)
        todayDinner = now.replace(hour=18,minute=30,second=0,microsecond=0)

        if now > todayEarly:
            if (now < todayAft):
                print('Late afternoon case')
                b.set_light('Table','on',True)
                b.set_light('Table','ct',175)
            elif (now > todayDinner) and (now < todayLate):
                print('Dinner case')
                b.set_light('Table','on',True)
                b.set_light('Table','ct',315)
            elif (now > todayLate) and (now < todayLateLim):
                print('Late Case')
                b.set_light('Table','on',True)
                b.set_light('Table','ct',400)

    else:
        print('Sending Hue Update to turn off')
            
        b.set_light('Table','on',False)



def setup_database():
    """Set up the database for storing the data sent by mqtt"""
    databasePath =  os.environ.get("SQL_PATH")
    databasePath = str(databasePath)+"/mqtt_indoor.sqlite"
    
    conn = None
    try:
        conn = sqlite3.connect(databasePath)
    except Error as e:
        print(e)

    return conn


def createTable(databasePath):
    """Make the SQLite table if it doesn't exist"""
    sql_create_weatherData_table = 'CREATE TABLE IF NOT EXISTS weatherData (id integer PRIMARY KEY,timedat text NOT NULL, temperature text, humidity text, pressure text, altitude text);'
    conn = setup_database()

    
    # create table
    if conn is not None:
        c = conn.cursor()
        c.execute(sql_create_weatherData_table)
    else:
        print("Error! cannot create the database connection.")



    
def insert_database(sqlMsg):
    """Save the weather data into a database"""

    databasePath =  os.environ.get("SQL_PATH")
    databasePath = str(databasePath)+"/mqtt_indoor.sqlite"
    
    conn = sqlite3.connect(databasePath)
    
    
    sql  ='INSERT INTO weatherData (timedat, temperature, humidity, pressure, altitude) values(?,?,?,?,?)'

    cur = conn.cursor()
    cur.execute(sql, sqlMsg) 
    conn.commit()

    



def main():
    # Start the Prometheus server
    start_http_server(8000)
    # Setup the SQLlite database
    databasePath = setup_database()
    createTable(databasePath)

    # Start mqtt client
    mqtt_client = mqtt.Client(MQTT_CLIENT_ID)
    mqtt_client.username_pw_set(MQTT_USER, MQTT_PASSWORD)

    # Specify what programs to call when mqtt conditions are met
    mqtt_client.on_connect = on_connect
    mqtt_client.on_message = on_message
    
    # Setup mqtt on a port
    mqtt_client.connect(MQTT_ADDRESS, 1883)
    # Keep running forever
    mqtt_client.loop_forever()


if __name__ == '__main__':
    main()
