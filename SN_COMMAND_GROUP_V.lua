Data = "4F4646"                -- Message, "OFF" in hex
Port = "15"                    -- The port number
AppEUI = "70b3d57ed000ac4a"
DevEUI = "8639323559379194"
HexID = "636f6e32"             -- The Connector Id that identifies the Broker/WebSocket/LoRaServer you want to use to send your message
Reference = ""                 -- When this particular transmission has been received. Can be left empty
Confirm = false                -- Allows the reference of the comunication
Command = ""                   -- A string that is used to filter determinated messages. It can be left empty
  	
resiot_debug("Sending command to node\n")
  
Error = resiot_tx(Data, Port, DevEUI, AppEUI, HexID, Reference, Confirm, Command)
if Error ~= "" then
    	-- error
    	resiot_debug(Error)
end
