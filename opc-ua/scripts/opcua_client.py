from asyncua import Client
import asyncio

class OPCUA_Client:
    def __init__(self, ip_address):
        if not self.is_valid_ip(ip_address):
            print("Invalid IP address format. Please enter a valid IP address.")
            exit()
        else:
            self.ip_address = ip_address

    # Check if IP is valid
    def is_valid_ip(ip):
        parts = ip.split('.')
        if len(parts) != 4:
            return False
        for part in parts:
            if not part.isdigit():
                return False
            if int(part) < 0 or int(part) > 255:
                return False
        return True


    # Run the client connection
    async def connect(self):
        async with Client(url='opc.tcp://%s:4840' % self.ip_address) as client:
            while True:
                # Do something with client
                node = client.get_node('i=85')
                value = await node.read_value()

    # Add a subscription to the client
        # Function to monitor temp, current, and voltage ?


if __name__ == "__main__":
    OPCUA_Client.connect(input("Enter the IP address of the OPC UA server: "))