import json
import boto3
import os

dynamodb = boto3.client("dynamodb")

def lambda_handler(event, context):
    print(f"Event object: {json.dumps(event)}")  # Debugging information

    try:
        # Parse the body and check for "payload"
        action = event["body"]["action"]
        payload = event["body"]["payload"]

        if action != "broadcast" or not payload:
            return {
                "statusCode": 400,
                "body": json.dumps("Invalid request: missing action or payload")
            }

        # Extract details from the payload
        message_type = payload["type"]  # Either 'sensor_data'

        paginator = dynamodb.get_paginator("scan")
        connection_ids = []

        apigatewaymanagementapi = boto3.client(
            "apigatewaymanagementapi",
            endpoint_url=f"https://{event['requestContext']['domainName']}/{event['requestContext']['stage']}"
        )

        # Scan the DynamoDB table to get all connection IDs
        for page in paginator.paginate(TableName=os.environ["WEBSOCKET_TABLE"]):
            connection_ids.extend(page["Items"])

        sender_connection_id = event["requestContext"]["connectionId"]

        # Broadcast the message to all clients except the sender
        for connection in connection_ids:
            connection_id = connection["connectionId"]["S"]
            if connection_id != sender_connection_id:
                try:
                    print(f"Sending {message_type} to connection {connection_id}")
                    apigatewaymanagementapi.post_to_connection(
                        Data=json.dumps({
                            "payload": payload,
                        }),
                        ConnectionId=connection_id
                    )
                except apigatewaymanagementapi.exceptions.GoneException:
                    print(f"Connection {connection_id} is gone.")

        return {
            "statusCode": 200,
            "body": json.dumps(f"{message_type} broadcasted successfully")
        }

    except Exception as e:
        print(f"Error: {e}")
        return {
            "statusCode": 500,
            "body": json.dumps(f"Internal server error: {str(e)}")
        }