import platform
import time
import asyncio
from bleak import BleakClient
from datetime import datetime
import csv

#TAG_MAC = 'FD:7B:CF:78:45:F9'
#TAG_UUID = '78FA9F97-8865-8214-4C67-0330C0761914'

TAG_MAC = 'DA:34:10:BC:D4:C5'
TAG_UUID = '82C04618-9C5A-2BA6-9D8B-B31876288389'

TEMPERATURE_1_UUID = '00001000-5468-616E-6B73-4D6F6D446164'
TEMPERATURE_2_UUID = '00002000-5468-616E-6B73-4D6F6D446164'
TEMPERATURE_3_UUID = '00003000-5468-616E-6B73-4D6F6D446164'
TEMPERATURE_4_UUID = '00004000-5468-616E-6B73-4D6F6D446164'


header = ['time', 'T1', 'T2', 'T3', 'T4']
data = []

with open('temperature.csv', 'w', encoding='UTF8', newline='\n') as file:
    writer = csv.writer(file)

    # write the header
    writer.writerow(header)
    file.close()

async def run(address):
    async with BleakClient(address) as client:
        print("SUCCESFULLY CONNECTED")

        while client.is_connected:
            now = datetime.now().strftime("%H:%M:%S")
            data.append(now)

            value_1 = bytes(await client.read_gatt_char(TEMPERATURE_1_UUID))
            temperature_1 = int.from_bytes(value_1, 'little')/1000
            data.append(temperature_1)
            print('Temperature 1: %.3f' %temperature_1)

            value_2 = bytes(await client.read_gatt_char(TEMPERATURE_2_UUID))
            temperature_2 = int.from_bytes(value_2, 'little') / 1000
            data.append(temperature_2)
            print('Temperature 2: %.3f' %temperature_2)

            value_3 = bytes(await client.read_gatt_char(TEMPERATURE_3_UUID))
            temperature_3 = int.from_bytes(value_3, 'little') / 1000
            data.append(temperature_3)
            print('Temperature 3: %.3f' %temperature_3)

            value_4 = bytes(await client.read_gatt_char(TEMPERATURE_4_UUID))
            temperature_4 = int.from_bytes(value_4, 'little') / 1000
            data.append(temperature_4)
            print('Temperature 4: %.3f' %temperature_4)
            print('')

            with open('temperature.csv', 'a', encoding='UTF8', newline='\n') as file:
                writer = csv.writer(file)

                # write the data
                writer.writerow(data)
                file.close()

            data.clear()

addr = (
    TAG_MAC if platform.system() != "Darwin"
    else TAG_UUID
)
loop = asyncio.get_event_loop()
loop.run_until_complete(run(addr))
