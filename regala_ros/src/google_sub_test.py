import os
from google.cloud import pubsub_v1
from concurrent.futures import TimeoutError

credentials_path = './regala_new.json'
os.environ['GOOGLE_APPLICATION_CREDENTIALS'] = credentials_path

timeout = 5.0

subscriber = pubsub_v1.SubscriberClient()
subscription_path = 'projects/project-regala/subscriptions/recording-cue-sub'

def callback(message):
    print('Received message : {}'.format(message))
    print('data : {}'.format(message.data))

    if message.attributes:
        print("Attributes")
        for key in message.attributes:
            value = message.attributes.get(key)
            print("key : {}, value : {}".format(key, value))
    message.ack()

streaming_pull_future = subscriber.subscribe(subscription_path, callback=callback)
print('Listening for message on {}'.format(subscription_path))

with subscriber:
    try:
        streaming_pull_future.result()
    except TimeoutError:
        streaming_pull_future.cancel()
        streaming_pull_future.result()