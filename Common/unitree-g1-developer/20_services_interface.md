# Services Interface

> 出典: https://support.unitree.com/home/en/G1_developer/services_interface

The software interface services of the G1 are similar to those of the Go2 、 B2 and H1 robots, supporting both the publish/subscribe and request/response communication modes.

Publish/Subscribe: The receiving party subscribes to a specific message, and the sending party sends messages to the receiving party based on the subscription list. This mode is mainly used for medium-to-high frequency or continuous data exchange.

Request/Response: This mode follows a question-and-answer pattern where data retrieval or operations are performed through requests. It is used for low-frequency or functionality-switching data exchange.

The official unitree_sdk2 provides optimized encapsulation for the request/response part, enabling function-based calls.

For detailed service interfaces, please refer to the sub-documentation.