% Copyright 2006-2015 Coppelia Robotics GmbH. All rights reserved. 
% marc@coppeliarobotics.com
% www.coppeliarobotics.com
% 
% -------------------------------------------------------------------
% THIS FILE IS DISTRIBUTED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
% WARRANTY. THE USER WILL USE IT AT HIS/HER OWN RISK. THE ORIGINAL
% AUTHORS AND COPPELIA ROBOTICS GMBH WILL NOT BE LIABLE FOR DATA LOSS,
% DAMAGES, LOSS OF PROFITS OR ANY OTHER KIND OF LOSS WHILE USING OR
% MISUSING THIS SOFTWARE.
% 
% You are free to use/modify/distribute this file for whatever purpose!
% -------------------------------------------------------------------
%
% This file was automatically created for V-REP release V3.2.3 rev4 on December 21st 2015

% This example illustrates how to execute complex commands from
% a remote API client. You can also use a similar construct for
% commands that are not directly supported by the remote API.
%
% Load the demo scene 'remoteApiCommandServerExample.ttt' in V-REP, then 
% start the simulation and run this program.
%
% IMPORTANT: for each successful call to simxStart, there
% should be a corresponding call to simxFinish at the end!

function complexCommandTest()

	function retData=getCmdString(id,cnt,data)
		l=12+length(data);
		retData=vrep.simxPackInts([id cnt l]);
		retData=[retData data];
	end
	
	function replyData=waitForCmdReply(cnt)
		while true
			[result string]=vrep.simxReadStringStream(clientID,'repliesToRemoteApiClient',vrep.simx_opmode_streaming);
			if (result==vrep.simx_return_ok)
				while (length(string)~=0)
					headerPacked=string(1:12);
					header=vrep.simxUnpackInts(headerPacked);
					if (cnt==header(2))
						replyData='';
						if header(3)>12
							replyData=string(13:header(3));
						end
						return;
					end
					if header(3)>=length(string)
						string='';
					else
						string=string(header(3)+1:length(string));
					end
				end
			end
		end
	end

	disp('Program started');
	% vrep=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
	vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
	vrep.simxFinish(-1); % just in case, close all opened connections
	clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

	if (clientID>-1)
		disp('Connected to remote API server');
		
		% Commands are send via the string signal 'commandsFromRemoteApiClient'.
		% Commands are simply appended to that string signal
		% Each command is coded in following way:
		% 1. Command ID (integer, 4 chars)
		% 2. Command counter (integer, 4 chars). Simply start with 0 and increment for each command you send
		% 3. Command length (integer, includes the command ID, the command counter, the command length, and the additional data (i.e. command data))
		% 4. Command data (chars, can be of any length, depending on the command)
		%
		% Replies are coded in a same way. An executed command should reply with the same command counter.
		% 
		% Above simple protocol is just an example: you could use your own protocol! (but it has to be the same on the V-REP side)
		
		% 1. First send a command to display a specific message in a dialog box:
		cmdID=1; % this command id means: 'display a text in a message box'
		cmdCnt=0;
		cmdData='Hello world!';
		stringToSend=getCmdString(cmdID,cmdCnt,cmdData);
		vrep.simxWriteStringStream(clientID,'commandsFromRemoteApiClient',stringToSend,vrep.simx_opmode_oneshot);
		replyData=waitForCmdReply(cmdCnt) % display the reply from V-REP (in this case, just a string)

		% 2. Now create a dummy object at coordinate 0.1,0.2,0.3:
		cmdID=2; % this command id means: 'create a dummy at a specific coordinate with a specific name'
		cmdCnt=cmdCnt+1;
		cmdData=['MyDummyName' vrep.simxPackFloats([0.1 0.2 0.3])];
		stringToSend=getCmdString(cmdID,cmdCnt,cmdData);
		vrep.simxWriteStringStream(clientID,'commandsFromRemoteApiClient',stringToSend,vrep.simx_opmode_oneshot);
		replyData=waitForCmdReply(cmdCnt);
		dummyHandle=vrep.simxUnpackInts(replyData) % display the reply from V-REP (in this case, the handle of the created dummy)
		
		% Now close the connection to V-REP:	
		vrep.simxFinish(clientID);
	else
		disp('Failed connecting to remote API server');
	end
	vrep.delete(); % call the destructor!
	
	disp('Program ended');
end
