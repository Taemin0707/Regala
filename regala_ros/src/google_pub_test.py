import os
from google.cloud import pubsub_v1

credentials_path = './regala_new.json'
os.environ['GOOGLE_APPLICATION_CREDENTIALS'] = credentials_path

publisher = pubsub_v1.PublisherClient()
topic_path = 'projects/project-regala/topics/recording-cue'

data = 'Test Message'
data = data.encode('utf-8')
attributes = {
    'user_id' : '777',
    'user_name' : 'Taemin',
    'time' : '30'
}

future = publisher.publish(topic_path, data, **attributes)
print('published message id {}'.format(future.result()))