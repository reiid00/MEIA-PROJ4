from flask import Flask, request
import threading

app = Flask(__name__)
pending_orders = []
pending_orders_lock = threading.Lock()

REQUIRED_PENDING_ORDER_ATTRIBUTES = [
    'order_id', 'details.customer_name', 'details.item', 'details.quantity',
    'details.dispatcher_location', 'details.customer_location', 'details.qrcode'
]

@app.route('/pending_orders', methods=['GET'])
def get_pending_orders():
    with pending_orders_lock:
        return {'pending_orders': pending_orders}, 200

@app.route('/pending_orders', methods=['POST'])
def add_pending_order():
    order_details = request.get_json()

    # Validate the order details
    for attr in REQUIRED_PENDING_ORDER_ATTRIBUTES:
        if attr not in order_details:
            return {'error': f'Missing required attribute: {attr}'}, 400

    with pending_orders_lock:
        pending_orders.append(order_details)

    return '', 200

if __name__ == '__main__':
    app.run(port=5000)

    
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