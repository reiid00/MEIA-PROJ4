import threading

from common.config import OrderStatus
from flask import Flask, request

app = Flask(__name__)
pending_orders = []
history_orders = []
pending_orders_lock = threading.Lock()
history_orders_lock = threading.Lock()

REQUIRED_PENDING_ORDER_ATTRIBUTES = [
    'order_id', 'details'
]

REQUIRED_PENDING_ORDER_DETAILS_ATTRIBUTES = [
    'customer_name', 'item', 'quantity',
    'dispatcher_location', 'customer_location', 'qrcode'
]

@app.route('/pending_orders', methods=['GET'])
def get_pending_orders():
     with pending_orders_lock:
        return {'pending_orders': pending_orders}, 200

@app.route('/history_orders', methods=['GET'])
def get_history_orders():
     with history_orders_lock:
        return {'history_orders': history_orders}, 200

@app.route('/update_order_status', methods=['POST'])
def update_order_status():
    data = request.get_json()
    order_id = data.get('order_id')
    status = data.get('status')

    if not order_id or not status:
        return {'error': 'Missing required attributes: order_id or status'}, 400

    with history_orders_lock:
        updated_order = None
        for order in history_orders:
            if order['order_id'] == order_id:
                order['status'] = status
                updated_order = order
                break

        if updated_order:
            return {'message': 'Order status updated successfully', 'updated_order': updated_order}, 200
        else:
            return {'error': f'Order {order_id} not found'}, 404

@app.route('/next_pending_order', methods=['GET'])
def get_next_pending_order():
    with pending_orders_lock:
        if pending_orders:
            # Remove the first pending order from the list and return it
            pending_order = pending_orders.pop(0)
            with history_orders_lock:
                order = pending_order.copy()
                order['status'] = OrderStatus.TO_BE_ASSIGNED.value
                history_orders.append(order)
            return {'next_pending_order': pending_order}, 200
        return {'error': 'There are no pending orders'}, 400

@app.route('/pending_orders', methods=['POST'])
def add_pending_order():
    order_details = request.get_json()

    # Validate the order details
    for attr in REQUIRED_PENDING_ORDER_ATTRIBUTES:
        if attr not in order_details:
            return {'error': f'Missing required attribute: {attr}'}, 400
        details = order_details["details"]
        for detail_attr in REQUIRED_PENDING_ORDER_DETAILS_ATTRIBUTES:
            if detail_attr not in details:
                return {'error': f'Missing required attribute: {detail_attr}'}, 400

    with history_orders_lock:
        for order in history_orders:
            if order['order_id'] == order_details["order_id"]:
                return {'error': f'Order {order_details["order_id"]} already exists'}, 400

    with pending_orders_lock:
        for order in pending_orders:
            if order['order_id'] == order_details["order_id"]:
                return {'error': f'Order {order_details["order_id"]} already exists in pending orders'}, 400
        pending_orders.append(order_details)

    return '', 200

if __name__ == '__main__':
    app.run(port=5001)
    
# Pending Order JSON Template:
# {
#     "order_id": 1,
#     "details": 
#     {
#           "customer_name": "John Doe",
#           "item": "Example Item",
#           "quantity": 2,
#           "dispatcher_location": {
#               "address": "123 Main St",
#               "latitude": 41.50,
#               "longitude": -10.150
#           },
#           "customer_location": {
#               "address": "123 Main St",
#               "latitude": 41.50,
#               "longitude": -10.150
#           },
#           "qrcode": "1234567890"
#     }
# }