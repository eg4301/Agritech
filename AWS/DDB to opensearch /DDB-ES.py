import boto3
import requests
from requests_aws4auth import AWS4Auth
from dynamodb_json import json_util as json

region = 'us-east-2' # e.g. us-east-1
service = 'es'
credentials = boto3.Session().get_credentials()
awsauth = AWS4Auth(credentials.access_key, credentials.secret_key, region, service, session_token=credentials.token)

host = 'https://search-rac-sensors-eg4301-dz4sukptjhkzoefd7nte6e2unu.us-east-2.es.amazonaws.com' # the OpenSearch Service domain, e.g. https://search-mydomain.us-west-1.es.amazonaws.com
index = 'rac_sensors'
datatype = '_doc'
url = host + '/' + index + '/' + datatype + '/'

headers = { "Content-Type": "application/json" }

def handler(event, context):
    count = 0
    for record in event['Records']:
        # Get the primary key for use as the OpenSearch ID
        id = record['dynamodb']['Keys']['timestamp']['S']

        if record['eventName'] == 'REMOVE':
            r = requests.delete(url + id, auth=awsauth)
        else:
            document = {
              "timestamp": record['dynamodb']['NewImage']['timestamp']['S'],
              "MAC": record['dynamodb']['NewImage']['MAC']['N'],
              "temperature": record['dynamodb']['NewImage']['temperature']['N'],
              "conductivity": record['dynamodb']['NewImage']['conductivity']['N'],
              "pH": record['dynamodb']['NewImage']['pH']['N']
            }
            r = requests.put(url + id, auth=awsauth, json=document)
        count += 1
    return str(count) + ' records processed.'
