//  APP2 examen formatif pratique
//  Reponse a la question Q5
//  Philippe Mabilleau ing.	mai 2021

var net = require('net');

var port = 9000; //The port that the server is listening on
var host = '10.0.10.10'; // adresse IP du module Argon

client = net.connect(port, host);
client.on('connect', function() { //Don't receive until we're connected
    client.on('data', function(rcv) {
		console.log(rcv.toString()); // Impression des valeurs reçues
    });
});
