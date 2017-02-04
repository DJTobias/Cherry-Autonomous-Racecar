#! /usr/bin/python

# Author: Ryan Dellana
# Date Created: Feb. 16, 2014
# Updated: December 20, 2016
# Note: This version was created for Daniel Tobias

"""
Dependencies:
sudo apt-get install python-pygame
sudo apt-get install festlex-cmu
sudo apt-get install ubuntu-restricted-extras
sudo dpkg-reconfigure libdvd-pkg
# restart computer

mirage_sound_out provides a simple way of playing sounds that come from outside sources (mp3, 3rd party text-to-speech, etc.).
mirage_sound_out was created as an alternative to the soundplay system that comes with ros hydro.
Unlike soundplay, this node is not suitable for use by multiple clients (they'll step on each other).

If it recieves a new sound while one is still playing, it'll just play them concurrently on separate audio channels.
Unless all 8 channels are already in use in which case it'll just drop the sound.
There is no queue.
In the future I'd like to have timing info passed along with the sounds in a simlar way as motor commands are sent.
So you'd pass it a schedule of sounds to play, consisting of a list of tuples each containing a sound and it's start time.
When processing such a list, this node could look ahead to the synthesized text portions and start generating their files
before the scheduled play time to reduce lag.

        Useful pygame.mixer.Sound properties.
        .play               -- begin sound playback
        .stop               -- stop sound playback
        .fadeout            -- stop sound playback after fading out
        .set_volume         -- set the playback volume for this Sound
        .get_volume         -- get the playback volume
        .get_num_channels   -- count how many times this Sound is playing
        .get_length         -- get the length of the Sound
        .get_raw            -- return a bytestring copy of the Sound samples
"""

import sys, os, time, json, shutil, copy
import numpy
import pygame.mixer
from pyfestival import Festival
import rospy
from std_msgs.msg import String

class mirage_sound_out(object):

    """ festival_cache_path: folder where pre-synthesized mp3 files will be stored.
        sound_effects_path: folder where you keep your sound effect files.
    """
    def __init__(self, festival_cache_path, sound_effects_path):
        pygame.mixer.init(channels=6)   
        self._festival_cache_path = festival_cache_path
        self._festival_tmp_cache_path = "/tmp/mirage_festival_tmp/"
        if not os.path.exists(self._festival_tmp_cache_path):
            os.makedirs(self._festival_tmp_cache_path)
        self._sound_effects_path = sound_effects_path
        self._sounds_preloaded = {} # TODO Not currently used.
        self._fest = Festival()
        self._load_cache()
        self._timestamp = time.time()
        self._get_runtime_ts()
        self._enforce_consistency()
        self._prune_cache()
        self._save_cache()
        print "finished running self.__init__"

    def shutdown(self):
        print 'shutdown routine...'
        self._save_cache()
        #pygame.mixer.stop()
        shutil.rmtree(self._festival_tmp_cache_path)
        pygame.mixer.quit()

    """ Automatically creates a cache file if there isn't already one. """
    def _load_cache(self):
        c_path = self._festival_cache_path + "festival_cache_index.json"
        if not os.path.exists(c_path): # create_cache
            self._festival_cache_index = { 'cache':{}, 'newest_cache_id':0, 'cache_max_size':100*1024, 'cache_size':0,
                                           'save_cache_after_n_updates':50, 'cache_updates_since_last_save':0, 
                                           'runtime_clock':0, 'synth_max_chars':150, 'prune_after':24*60*60 }
        else:
            in_file = open(self._festival_cache_path + "festival_cache_index.json") # TODO error handling?
            self._festival_cache_index = json.load(in_file)
        

    """ Make sure that all files specified in index are present in cache and vice versa. """
    # TODO Need to test this feature. Try removing some of the files from the cache and see if it adapts.
    #   Also try manually corrupting the index by adding references to files that aren't there.
    def _enforce_consistency(self):
        print "_enforce_consistency()"
        cache_files = os.listdir(self._festival_cache_path)
        ammended_cache = copy.deepcopy(self._festival_cache_index)
        ammended_cache['cache'] = {}
        for k in self._festival_cache_index['cache'].keys():
            ammended_cache['cache'][k] = copy.deepcopy(self._festival_cache_index['cache'][k])
            if ammended_cache['cache'][k]['fname'] != "" and ammended_cache['cache'][k]['fname'] not in cache_files:
                print k, "not found in cache. Removing from index."
                ammended_cache['cache'][k]['fname'] = ""
                ammended_cache['cache_size'] -= ammended_cache['cache'][k]['size']
                ammended_cache['cache'][k]['size'] = 0
        self._festival_cache_index = ammended_cache
        index_files = [self._festival_cache_index['cache'][k]['fname'] for k in self._festival_cache_index['cache'].keys()]
        for f in cache_files:
            if f not in index_files and f.count("festival_cache_index") == 0:
                print f, "not found in index. Removing from cache."
                try:
                    os.remove(self._festival_cache_path + f)
                except:
                    print "ERROR: os.remove(" + self._festival_cache_path + f + ")"
                    pass

    def _prune_cache(self):
        for k in self._festival_cache_index['cache'].keys():
            item = self._festival_cache_index['cache'][k]
            if item['size'] == 0 and (self._get_runtime_ts() - item['runtime_stamp'] > self._festival_cache_index['prune_after']):
                del self._festival_cache_index['cache'][k]
                    
    def _save_cache(self):
        print "_save_cache():" #, self._festival_cache_index
        output_file = open(self._festival_cache_path + "festival_cache_index.json", "wb")
        if output_file is not None:
            json.dump(self._festival_cache_index, output_file, indent=4)
        else:
            print "ERROR: output_file is None"
        output_file.close()

    def say(self, txt):
        print "say(", txt, ")"
        if len(txt) <= self._festival_cache_index['synth_max_chars']:
            cln_txt = self._clean_phrase(txt)
            if cln_txt not in self._festival_cache_index['cache'].keys(): # First time ever said.
                self._festival_cache_index['cache'][cln_txt] = { 'fname':"", 'size':0.0, 'count':0, 'runtime_stamp':0, 'delete_score':0 }
            elif self._festival_cache_index['cache'][cln_txt]['fname'] == "" and self._festival_cache_index['cache'][cln_txt]['count'] >= 1: # Second time ever been said.
                self._cache_phrase(txt)
            return self._say(txt)
        return 0.0

    """ updates cache index runtime_stamp and than returns it's new value """
    def _get_runtime_ts(self):
        current_time = time.time()
        time_elapsed = current_time - self._timestamp
        self._festival_cache_index['runtime_clock'] = self._festival_cache_index['runtime_clock'] + time_elapsed
        self._timestamp = current_time
        print "runtime_clock:", str(self._festival_cache_index['runtime_clock'])
        return self._festival_cache_index['runtime_clock']

    """ Remove all chars other than number, letter, and space """
    def _clean_phrase(self, txt):
        phrase = txt.lower()
        out = ""
        for ch in phrase:
            if (ord(ch) >= ord('a') and ord(ch) <= ord('z')) or (ord(ch) == ord(' ')) or (ord(ch) >= ord('0') and ord(ch) <= ord('9')):
                out += ch
        return out

    def _cache_phrase(self, txt):
        print "_cache_phrase(", txt, ")"
        cache_id = self._festival_cache_index['newest_cache_id'] + 1
        cache_id_str = "cid" + str(cache_id) + ".wav"
        _dest = self._festival_cache_path + cache_id_str
        try:
            self._fest.wave(text=txt, dest=_dest)
        except:
            print "ERROR: text2wave. txt=", txt, "dest=", _dest
            return False
        self._festival_cache_index['newest_cache_id'] = cache_id
        entry = self._festival_cache_index['cache'][self._clean_phrase(txt)]
        entry['fname'] = cache_id_str
        entry['size'] = int(os.stat(_dest).st_size / 1024) # store size as truncated kilobytes.
        self._festival_cache_index['cache_size'] += entry['size']
        self._regulate_cache_size()
        return True

    def _regulate_cache_size(self):
        print "_regulate_cache_size()"
        if self._festival_cache_index['cache_size'] > self._festival_cache_index['cache_max_size']:
            print "cache exceeded maximum size of", self._festival_cache_index['cache_max_size'], "kilobytes. reducing..."
            cached = []
            for k in self._festival_cache_index['cache'].keys(): # vvv This is going to get slow eventually.
                if self._festival_cache_index['cache'][k]['size'] != 0:
                    cached.append(self._festival_cache_index['cache'][k])
            tstmps = [e['runtime_stamp'] for e in cached]
            sizes = [e['size'] for e in cached]
            counts = [e['count'] for e in cached]
            median_tstmp = numpy.median(tstmps)
            print "median_tstmp =", median_tstmp
            median_size = numpy.median(sizes)
            print "median_size =", median_size
            median_count = numpy.median(counts)
            print "median_count =", median_count
            std_dev_tstmp = numpy.std(tstmps)
            print "std_dev_tstmp =", std_dev_tstmp
            std_dev_size = numpy.std(sizes)
            print "std_dev_size =", std_dev_size
            std_dev_count = numpy.std(counts)
            print "std_dev_count =", std_dev_count
            for e in cached:
                std_devs_before_med_tstmp = (median_tstmp - e['runtime_stamp']) / std_dev_tstmp
                std_devs_above_med_size = (e['size'] - median_size) / std_dev_size
                std_devs_below_med_count = (median_count - e['count']) / std_dev_count
                e['delete_score'] = 0.75*std_devs_before_med_tstmp + 0.15*std_devs_above_med_size + 0.10*std_devs_below_med_count
            cached_sorted = sorted(cached, key = lambda k: k['delete_score'])
            while (self._festival_cache_index['cache_size'] > self._festival_cache_index['cache_max_size']):
                delete_me = cached_sorted[-1]
                print "deleting: fname =", delete_me['fname'], "age =", delete_me['runtime_stamp'], "size =", delete_me['size'], "count =", delete_me['count']
                try:
                    os.remove(self._festival_cache_path + delete_me['fname'])
                except:
                    print "ERROR: os.remove(" + self._festival_cache_path + delete_me['fname'] + ")"
                    return False
                self._festival_cache_index['cache_size'] -= delete_me['size']
                delete_me['size'] = 0
                delete_me['fname'] = ""
                cached_sorted.remove(delete_me)
        return True

    def _say(self, txt):
        cln_txt = self._clean_phrase(txt)
        entry = self._festival_cache_index['cache'][cln_txt]
        entry['count'] += 1
        entry['runtime_stamp'] = self._get_runtime_ts()
        seconds = 0.0
        if entry['fname'] != "": # play from cache
            print 'playing from cache'
            seconds = self.play(fname = entry['fname'], cached_phrase = True, tmp_cache = False)
        else: # slingshot through tmp directory (RAM)
            print 'playing from RAM'
            _dest = self._festival_tmp_cache_path + "tmp_sound.wav"
            try:
                self._fest.wave(text=txt, dest=_dest) # TODO I'm assumming this will just overwrite any older tmp_sound.wav
            except:
                print "ERROR: text2wave. txt=", txt, "dest=", _dest
                return
            seconds = self.play(fname = "tmp_sound.wav", cached_phrase = True, tmp_cache = True)
        self._festival_cache_index['cache_updates_since_last_save'] += 1
        if self._festival_cache_index['cache_updates_since_last_save'] >= self._festival_cache_index['save_cache_after_n_updates']:
            self._festival_cache_index['cache_updates_since_last_save'] = 0
            self._save_cache()
        return seconds

    """ plays a sound specified by fname. If cached_phrase is set to True, then
        it expects a cacheid. If False, it assumes the sound to be in the sound_effects
        folder. If other sound(s) are already playing, it'll just overlap with them by 
        playing on another open channel (unless all chanels are busy). """
    def play(self, fname, cached_phrase=False, tmp_cache=False):
        path = ""
        if cached_phrase:
            if tmp_cache:
                path = self._festival_tmp_cache_path + fname
            else:
                path = self._festival_cache_path + fname
        else: 
            path = self._sound_effects_path + fname
        try:
            #if path not in self._sounds_preloaded.keys():
            #    self._sounds_preloaded[path] = pygame.mixer.Sound(path)
            channel = pygame.mixer.find_channel()
            if channel <> None:
                #channel.play(self._sounds_preloaded[path])
                #return self._sounds_preloaded[path].get_length()
                sound = pygame.mixer.Sound(path)
                channel.play(sound)
                return sound.get_length()
        except:
            print "ERROR: Failed to play: " + path
        return 0.0

    def stop(self):
        pygame.mixer.stop()

    def pause(self):
        pygame.mixer.pause()

    def resume(self):
        pygame.mixer.unpause()

    
class SoundServer(object):

    def __init__(self):
        rospy.init_node('sound_server', anonymous=False)
        rospy.on_shutdown(self._shutdown)
        while (rospy.get_rostime() == 0.0):
            pass
        rospy.Subscriber("/sound_server/speech_synth", String, self.cb_speech_synth)
        rospy.Subscriber("/sound_server/play_sound_file", String, self.cb_play_sound_file)
        self.pub_status = rospy.Publisher("/sound_server/status", String)
        self.s = mirage_sound_out(festival_cache_path="/home/ubuntu/Music/mirage_engrams/festival_cache/", sound_effects_path="/home/ubuntu/Music/mirage_engrams/sound_effects/")
        rospy.sleep(2.0) # Startup time.
        rospy.loginfo("mirage_sound_out ready")
        rospy.spin()

    def _shutdown(self):
        if self.s is not None:
            self.s.shutdown()

    def cb_speech_synth(self, data):
        if data is not None and data.data != "": # TODO add more input validation.
            sound_seconds = self.s.say(data.data)
            self.pub_status.publish(String(str(sound_seconds)))
        
    def cb_play_sound_file(self, data):
        if data is not None and data.data != "": # TODO add more input validation.
            sound_seconds = self.s.play(data.data)
            self.pub_status.publish(String(str(sound_seconds)))

if __name__ == '__main__':
    try:
        server = SoundServer()
    except rospy.ROSInterruptException:
        pass

