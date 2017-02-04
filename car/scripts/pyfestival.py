# -*- coding: utf-8 -*-
#
# Copyright (C) 2008 John Paulett
# All rights reserved.
#
# This software is licensed as described in the file COPYING, which
# you should have received as part of this distribution.

"""Python bindings for The Festival Speech Synthesis System.

http://code.google.com/p/pyfestival/
http://www.cstr.ed.ac.uk/projects/festival/
"""

__version__ = "0.2.0"
__all__ = [
    'Festival', 'FestivalServer', 'FestivalClient', 'say'
]
import os
import signal
import subprocess
import socket

class Festival(object):
   
    def __init__(self, port=None, language=None, heap=None):
        self._server = None
       
        self._festival_bin = "festival"
        self._text2wave_bin = "text2wave"
        self._port = port
        self._language = language
       
   
    def open(self):
        """Opens a connection to the server.
       
        Must be called before say() when using Festival in server mode.
        Server mode creates extra overhead when opening the connection,
        but is beneficial when making multiple calls to the festival engine.
        """
        if self._server != None:
            # if an instance of the server already exists, close it
            # before opening a new one
            self.close()
        self._server = FestivalServer()
        self._server.start()
       
    def close(self):
        """Stops the server and closes the connection.
       
        Should be called when finished using the program in order to free
        the port the server is running on and
        """
        if self._server != None:
            self._server.stop()
            self._server = None
       
    def say(self, text):
        """Orders Festival to speak the text provided.
       
        Action depends on whether the open() method was used prior
        to calling say().  If a server connection was created (i.e. by
        calling open()), then this method communicates with the server
        instance.  If open() was not callled, say() uses a separate
        instance for every invokation.
        """
        if self._server != None:
            # speak to the server via sockets
            self._say_server(text)
        else:
            # no server, so use a single instance of festival
            self._say_single(text)
       
    def say_file(self, filename):
        """Speaks the contents of a file.
       
        If there is an issue opening or reading the file, nothing will
        be spoken.
        """
        text = self._read_file(filename)
        if text != None:
            self.say(text)
       
    def wave(self, text=None, source=None, dest=None,
              wave_type=None, frequency=None, scale=None):
        """Uses Festival's text2wave program to generate a wave file object.
       
        Must contain either a text or source_file input (if both are given,
        text overrides the source_file.  Returns the stdout wav, unless a
        dest_file is given, in which case it will return None and save the wav
        in the dest_file.
       
        Options:
        text - the text to be spoke in the wav
        source - the file to be read in
        dest - the output file, if not specificied, output is returned
        wave_type - the output waveform type (alaw, ulaw, snd, aiff, riff,
                    nist, etc.  Defaults to the programs default (which is
                    riff).
        frequency - the output frequency.  Defaults to program's default.
        scale - the volume factor.  Defaults to program's default.
        """
        args = [self._text2wave_bin]
        if text == None:
            # check to make sure something will act as input
            if source == None:
                raise TooFewArgumentsError("Need an input value.")
            # have program read the source file
            args.append(source)
        else:
            # tts mode
            args.append("-")
        # append optional arguments
        if dest != None:
            args.append("-o "+dest)
        if wave_type != None:
            args.append("-mode "+wave_tye)
        if frequency != None:
            args.append("-F "+frequency)
        if scale != None:
            args.append("-scale "+scale)
        print args
        # opena connection to the file
        #p = subprocess.Popen( args,
        #                      stdin = subprocess.PIPE,
        #                      stdout = subprocess.PIPE,
        #                      stderr = subprocess.PIPE,
        #                      close_fds = True)
        #stdout, stderr = p.communicate(text)
        if dest <> None and text <> None:
            subprocess.call("echo \"" + text + "\" | " + self._text2wave_bin + " -o " + dest, shell=True)
        # only return a value if the file is not saved.
        if dest == None:
            return stdout
        else:
            return None
   
    def version(self):
        """Returns the version of Festival and of PyFestival.
        """
        args = [self._festival_bin, "--version"]
        p = subprocess.Popen( args,
                              stdin = subprocess.PIPE,
                              stdout = subprocess.PIPE,
                              stderr = subprocess.PIPE,
                              close_fds = True)
        stdout, stderr = p.communicate()
       
        stdout += "\n"
        stdout += "PyFestival version: " + __version__
        return stdout
   
    def _save_file(self, filename, contents, mode="w"):
        """Saves the contents to the filename.
       
        Mode indicates the mode with which the file should
        be opened ('w','wb','a', or 'ab').  If an error occurs when
        writing the file, a simple message is printed to stdout.
        """
        try:
            f = open(filename, mode)
            f.write(contents)
            # clean up the resource
            f.close()
        except Exception:
            print "Execption: "+filename+" could not be saved."
       
   
    def _read_file(self, filename, mode="r"):
        """Reads the contents of a file.
       
        The contents are returned, unless there is an error reading
        the file, in which case None is returned.  The mode specifies
        the mode to read the file in--should be 'r' or 'rb'.
        """
        # default the return value
        contents = None
        try:
            # open and read the file (asssumes that file will fit into memory"
            f = open(filename, mode)
            contents = f.read()
            # clean up resources
            f.close()
        except Exception:
            print "Execption: "+filename+" could not be read."
        return contents
           
   
    def _say_server(self, text):
        """Uses the Festival server to speak the text.
       
        A connection to the server must be open.
        """
        if self._server == None:
            raise ServerError("No server started")
       
           
           
    def _say_single(self, text):
        """Uses Festival tts mode and the command line.
       
        Does not bother using the server connection."""
        args = [self._festival_bin, "--tts", "-"]
       
        p = subprocess.Popen( args,
                              stdin = subprocess.PIPE,
                              stdout = subprocess.PIPE,
                              stderr = subprocess.PIPE,
                              close_fds = True)
        stdout, stderr = p.communicate(text)
       


class FestivalServer(object):
    def __init__(self):
        # initialize the process
        self._process = None
        #self._festival_bin = "festival" # TODO Added this dellanar04
        try:
            self.start()
        except ServerError, e:
            pass

    def start(self):
        args = [self._festival_bin, "--server"]
        self._process = subprocess.Popen( args,
                              stdin = subprocess.PIPE,
                              stdout = subprocess.PIPE,
                              stderr = subprocess.PIPE,
                              close_fds = True)
        stdout, stderr = self._process.communicate()
       
        if stderr.rstrip() == "socket: bind failed":
            raise ServerError(stderr.rstrip())
       
        #self._pid = os.spawnlp(os.P_NOWAIT, "festival","festival", "--server")
        #print self._pid
           
    def stop(self):
        """Kill the instance of the festival server.
       
        """
        if self.get_pid() != None:
            try:
                os.kill(self._pid, signal.SIGTERM)
            except OSError:
                print "Error killing the festival server"
        else:
            print "No festival server process to stop"
           
    def restart(self):
        self.stop()
        self.start()
   
    def get_pid(self):
        if self._process != None:
            return self._process.pid
        else:
            return None

class FestivalClient(object):
    def __init__(self, port=1314, host="localhost"):
        self._host = host
        self._port = port
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
       
    def open(self):
        self._sock.connect((self._host, self._port))
       
    def close(self):
        self._sock.close()
       
    def send(self, cmd):
        self._sock.send(cmd)
   
    def recv(self):
        data = self._sock.recv()
        return None
   
    def say(self, text):
        self.send('(SayText "%s"'%text)
   
   
class TooFewArgumentsError(Exception):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)
class ServerError(Exception):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)


def say(text):
    """Quick access for the impatient.
   
    Speaks the provided text.
    """
    #create a single instance to say this one phrase
    fest = Festival()
    fest.say(text)
    
