from flask import Flask, jsonify, request

app = Flask(__name__)
if __name__ == "main":
    app.run(debug=True)
