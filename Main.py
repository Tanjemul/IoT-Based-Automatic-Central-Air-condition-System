import serial
import datetime
import struct
import sqlite3
from datetime import datetime
import time 
import re
import math
import BlynkLib
from BlynkTimer import BlynkTimer

start_time = time.time()

conn = sqlite3.connect("/home/tj/iotdb.db")
cursor = conn.cursor()
# Setup Blynk
BLYNK_AUTH_TOKEN = 'CXtpb9VJwvyt3xT9WYR-RbZWDkNh5CgK'
# Initialize Blynk
blynk = BlynkLib.Blynk(BLYNK_AUTH_TOKEN)

control_mode=0
# Register virtual pin handler
@blynk.on("V6")
def v3_write_handler(value):
    control_mode= value[0]       

set_temp_n1=0
@blynk.on("V7")
def v7_write_handler(value):
    set_temp_n1= value[0]
 
set_temp_n2=0
@blynk.on("V8")
def v8_write_handler(value):
    set_temp_n2= value[0]

# Open serial port
ser = serial.Serial('/dev/ttyUSB0', 19200)
ser.reset_input_buffer()

controlled_temp = []

#Calculate adaptive_comfort_temperature
def calculate_act(ambient_temperature, relative_humidity, number_of_people):
    # Calculate the comfort temperature using the adaptive comfort model
    comfort_temperature = ambient_temperature + 0.31 * (25 - ambient_temperature) - 3.3 * (1 - math.exp(-0.026 * number_of_people)) - 0.28 * (ambient_temperature - 25) * (1 - math.exp(-0.1 * relative_humidity))
    return comfort_temperature


#Send controlled temperature
def send_controlled_temp(controlled_temp):
	controlled_temp_str = ', '.join(str(x) for x in controlled_temp)
	try:
		ser.reset_input_buffer()
		if ser.in_waiting == 0 :
			ser.write(controlled_temp_str.encode())
			ser.reset_output_buffer()  # Clear the output buffer

	except serial.SerialException as se:
		print("Serial port error:", str(se))
		message = [0.1]
		print(message)
		ser.reset_output_buffer()  # Clear the output buffer
	except struct.error as e:
		print("Struct unpacking error:", str(e))
		message = [0.1]
		print(message)
		ser.reset_output_buffer()  # Clear the output buffer
	except Exception as e:
		print("Error:", str(e))
		message = [0.1]
		print(message)
		ser.reset_output_buffer()  # Clear the output buffer

while True:
			try:
				if ser.in_waiting > 0:
					message = ser.readline().decode().strip()
					
					float_data = [float(x) for x in message.split(",")]
					counter = 0
					for num in float_data:
						if num >= 0:
							counter +=1

					ser.reset_input_buffer()
					print(float_data)
					
					#print("Received message: " + message)
					#send_controlled_temp(controlled_temp)
					##################################################
					
					if(len(float_data) == counter):
                    					
						cursor.execute("INSERT INTO firstnode (time_stamp,temperature,humidity,people) VALUES(?,?,?,?)", 
						(datetime.now(), float_data[0], float_data[1],float_data[2]))
								  
						cursor.execute("INSERT INTO secondnode (time_stamp,temperature,humidity,people) VALUES(?,?,?,?)", 
						(datetime.now(), float_data[3],float_data[4],float_data[5]))
						query1 = """
								  INSERT INTO avg_firstnode (time_stamp,avg_temp,avg_hum,avg_pep)
								  SELECT datetime('now'), AVG(temperature), AVG(humidity), AVG(people)
								  FROM (
									SELECT temperature, humidity, people
									FROM firstnode
									ORDER BY time_stamp DESC
									LIMIT 5
								  ) AS subquery;

								  """        
						cursor.execute(query1)
						  
						query2 = """
								  INSERT INTO avg_secondnode (time_stamp,avg_temp,avg_hum,avg_pep)
								  SELECT datetime('now'), AVG(temperature), AVG(humidity), AVG(people)
								  FROM (
									SELECT temperature, humidity, people
									FROM secondnode
									ORDER BY time_stamp DESC
									LIMIT 5
								  ) AS subquery;

								  """        
						cursor.execute(query2) 
						#cursor.execute("DELETE FROM firstnode")
						#cursor.execute("DELETE FROM avg_firstnode")
						#cursor.execute("DELETE FROM secondnode")
						#cursor.execute("DELETE FROM avg_secondnode")
						conn.commit() 
						
						cursor.execute("SELECT avg_temp, avg_hum FROM avg_firstnode ORDER BY time_stamp DESC LIMIT 1")
						avg_n1 = cursor.fetchone()
						avg_temp_n1 = avg_n1[0]
						avg_hum_n1 = avg_n1[1]
						
						#print(avg_n1[0])
						#print(avg_n1[1])
						
						cursor.execute("SELECT avg_temp, avg_hum FROM avg_secondnode ORDER BY time_stamp DESC LIMIT 1")
						avg_n2 = cursor.fetchone()
						avg_temp_n2 = avg_n2[0]
						avg_hum_n2 = avg_n2[1]
						
						#print(avg_n2[0])
						#print(avg_n2[1])
						print("Data array sent to the server successfully....")
												
						blynk.virtual_write(0, float_data[0])
						blynk.virtual_write(1, float_data[1])
						blynk.virtual_write(2, float_data[2])
						blynk.virtual_write(3, float_data[3])
						blynk.virtual_write(4, float_data[4])
						blynk.virtual_write(5, float_data[5])
						
						if control_mode==0:
						
							if set_temp_n1== 0 and set_temp_n2==0:
								controlled_temp = [24.00,24.00]
								send_controlled_temp(controlled_temp)
								
							elif set_temp_n1>0 and set_temp_n2==0:
								controlled_temp = [set_temp_n1,24.00]
								send_controlled_temp(controlled_temp)
								
							elif set_temp_n1==0 and set_temp_n2>0:
								controlled_temp = [24.00, set_temp_n2]
								send_controlled_temp(controlled_temp)
									
							elif set_temp_n1>0 and set_temp_n2>0:
								controlled_temp = [set_temp_n1, set_temp_n2]
								send_controlled_temp(controlled_temp)
							else:
								 controlled_temp = [24.00,24.00]
								 send_controlled_temp(controlled_temp)
									
						if control_mode==1:
							controlled_temp=[calculate_act(float_data[0],float_data[1],float_data[2]),calculate_act(float_data[3],float_data[4],float_data[5])]
							send_controlled_temp(controlled_temp)
								   
						blynk.run()
						time.sleep(.0001)
						
						end_time = time.time()
						
						#print(start_time-end_time)

							
			except serial.SerialException as se:
				print("Serial port error:", str(se))
				# Handle the serial port error as needed
				message = [0.0]
				ser.reset_input_buffer()  # Clear the input buffer
			except struct.error as e:
				print("Struct unpacking error:", str(e))
				# Handle the struct unpacking error as needed
				message = [0.0]
				ser.reset_input_buffer()  # Clear the input buffer
			except Exception as e:
				print("Error:", str(e))
				# Handle other exceptions as needed
				message = [0.0]
				ser.reset_input_buffer()  # Clear the input buffer
#cursor.close()	
conn.close()