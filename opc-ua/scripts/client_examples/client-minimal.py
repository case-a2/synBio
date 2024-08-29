import asyncio
import logging
from asyncua import Client, Node, ua

logging.basicConfig(level=logging.INFO)
_logger = logging.getLogger('asyncua')


class SubHandler:
    """
    Subscription Handler. To receive events from server for a subscription
    data_change and event methods are called directly from receiving thread.
    Do not do expensive, slow or network operation there. Create another
    thread if you need to do such a thing
    """

    def datachange_notification(self, node, val, data):
        print("New data change event", node, val)

    def event_notification(self, event):
        print("New event", event)
# async def browse_nodes(node: Node):
    

    # node_class = await node.read_node_class()
    # children = []
    # for child in await node.get_children():
    #     if await child.read_node_class() in [ua.NodeClass.Object, ua.NodeClass.Variable]:
    #         children.append(
    #             await browse_nodes(child)
    #         )
    # if node_class != ua.NodeClass.Variable:
    #     var_type = None
    # else:
    #     try:
    #         var_type = (await node.read_data_type_as_variant_type()).value
    #     except ua.UaError:
    #         _logger.warning('Node Variable Type could not be determined for %r', node)
    #         var_type = None
    # return {
    #     'id': node.nodeid.to_string(),
    #     'name': (await node.read_display_name()).Text,
    #     'cls': node_class.value,
    #     'children': children,
    #     'type': var_type,
    # }


async def task(loop):
    url = "opc.tcp://10.42.0.101:4840"
    # url = "opc.tcp://10.42.0.101:4840/freeopcua/server/"
    try:
        client = Client(url=url)
        client.set_user('admin')
        client.set_password('private')
        # client.set_security_string()
        await client.connect()


        _logger.info("Root node is: %r", client.nodes.root)
        _logger.info("Objects node is: %r", client.nodes.objects)

            # Node objects have methods to read and write node attributes as well as browse or populate address space
        _logger.info("Children of root are: %r", await client.nodes.root.get_children())


        # Client has a few methods to get proxy to UA nodes that should always be in address space such as Root or Objects
        root = client.nodes.root
        _logger.info("Objects node is: %r", root)

        # Node objects have methods to read and write node attributes as well as browse or populate address space
        _logger.info("Children of root are: %r", await root.get_children())

        # tree = await browse_nodes(client.nodes.objects)
        #
        # Now getting a variable node using its browse path 
        #myvar = await client.nodes.root.get_child("0:Objects")
        #obj = await client.nodes.root.get_child("Objects/2:MyObject")
        #_logger.info("myvar is: %r", myvar)

        # get a specific node knowing its node id
        var = client.get_node(ua.NodeId(85, 0))

        await client.get_namespace_array()
        # var = client.get_node("ns")
        print(var)
        #print(var.get_path())
        #await var.read_value() # get value of node as a DataValue object


        # subscribing to a variable node
        #handler = SubHandler()
        #sub = await client.create_subscription(10, handler)
        #handle = await sub.subscribe_data_change(myvar)
        await asyncio.sleep(0.1)

        # we can also subscribe to events from server
        #await sub.subscribe_events()
        # await sub.unsubscribe(handle)
        # await sub.delete()



    except Exception:
        _logger.exception('error')
    finally:
        await client.disconnect()

    


def main():
    loop = asyncio.get_event_loop()
    loop.set_debug(True)
    loop.run_until_complete(task(loop))
    loop.close()


if __name__ == "__main__":
    main()