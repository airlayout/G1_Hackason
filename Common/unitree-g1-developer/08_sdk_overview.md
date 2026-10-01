# SDK Overview

> 出典: https://support.unitree.com/home/en/G1_developer/sdk_overview

The current software version does not support GST video streaming yet.

G1 uses DDS as the message middleware, and the main data interaction adopts two modes: subscription/publish and request/response.

Subscription/Publish: The receiver subscribes to a message, and the sender sends messages to the receiver according to the subscription list. It is mainly used for medium-to-high frequency or continuous data interaction.

Request/Response: Question and answer mode, data acquisition or operation is achieved through requests. Used for data interaction at low frequency or function switching.

The calling method of the request response interface: API call, Functional call

API call: Similar to restapi, fill in the request content and request topic when sending the request, and accept the reply in the corresponding response topic. The reply adopts broadcast mode and determines the corresponding relationship between the request and the response according to the UUID.

Functional call: The syntactic sugar of API call mode, which encapsulates API calls into function calls to facilitate user use.

Development Kit:

unitree_sdk2: C++ development library