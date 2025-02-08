import boto3
import os
import json

dynamodb = boto3.client("dynamodb")

def lambda_handler(event, context):
    connectionId = event["requestContext"]["connectionId"]

    dynamodb.delete_item(
        TableName=os.environ["WEBSOCKET_TABLE"],
        Item={"connectionId": {"S": connectionId}}
    )

    return {}