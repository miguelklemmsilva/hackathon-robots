import json

def lambda_handler(event, context):
    print("Default handler invoked.")
    print("Event:", json.dumps(event))

    return {
        "statusCode": 200,
        "body": json.dumps({"message": "Unrecognized action"})
    }