package main

import (
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"flag"
	"fmt"
	"io/ioutil"
	"log"
	"math"
	"net/http"
	"net/url"
	"path/filepath"
	"strconv"
	"strings"
	"time"
)

// Robot must be exported so that json package can access it to encode
// all strings because they're going to be serialized to JSON anyways
type Robot struct {
	Name      string // the name of the robot
	User      string // the user logged in to the robot
	IP        string // the ip address of the robot
	X         string // x coordinate
	Y         string // y coordinate
	Level     string // floor the robot is on
	Alive     string // if the robot is currently active
	LastAlive string // epoch time since robot last pinged server
}

func (bot Robot) String() string {
	ret := ""
	ret += bot.Name + "\n"
	ret += "\t" + "User: " + bot.User + "\n"
	ret += "\t" + "IP: " + bot.IP + "\n"
	ret += "\t" + "Coordinates: (" + bot.X + ", " + bot.Y + ")\n"
	ret += "\t" + "Level: " + bot.Level + "\n"
	ret += "\t" + "Alive: " + bot.Alive + "\n"
	ret += "\t" + "Time Last Alive: " + bot.LastAlive + "\n"
	return ret
}

var robots []Robot

// command line args
var portNumber int
var file string
var debug bool
var insecure bool

var aliveTicker = time.NewTicker(10 * time.Second) // controls time between updates to who is alive
var aliveTimeout = int64(10)                       // (in seconds) if a robot isn't heard from in this time, it's not alive
var tokenTimeout = int64(300)                      // (in seconds) tokens expire after this amount of time
var password string                                // the password that .cmasskey stores
var hasher = sha256.New()                          // used to hash
var hashIterations = 1000                          // must be agreed upon by client and server
var timeLastSaved int64

/////////////
// ROUTING //
/////////////

// converts a string-returning function to a function that writes to
func serveBasicHTML(f func() string) func(http.ResponseWriter, *http.Request) {
	return func(w http.ResponseWriter, r *http.Request) {
		w.Header().Set("Access-Control-Allow-Origin", "*")
		fmt.Fprintf(w, f())
	}
}

//define functions for server endpoints
////		/update						called by robots with their info
////		/text							serves all robot info
////		/json							serves all robot info as json
////		/hosts						legacy support, serves robotname:IP
////		/hostsjson				legacy support, serves json of robotname:IP
////		/hostsalive				legacy support, serves robotname:IP of active robots
////		/hostsalivejson		legacy support, serves json of robotname:IP of active robots

func update(w http.ResponseWriter, r *http.Request) {
	query := r.URL.Query()
	splitIP := strings.Split(r.RemoteAddr, ":")
	requestIP := strings.Join(splitIP[:len(splitIP)-1], ":") //removes what's after the last ':' (done this way bc IP6)
	pdebug("IP of request: " + requestIP)
	message, valid := updateRobot(query, requestIP)
	if valid {
		save()
	}
	fmt.Fprintf(w, message)
}

func jsonFull() string {
	bytes, err := json.Marshal(robots)
	checkErr(err, "couldn't jsonify")
	return string(bytes)
}

func hosts() string {
	ret := ""
	for _, bot := range robots {
		ret += bot.Name + "\t\t"
		ret += bot.IP + "\n"
	}
	return ret
}

func hostsalive() string {
	ret := ""
	for _, bot := range robots {
		if bot.Alive == "true" {
			ret += bot.Name + "\t\t"
			ret += bot.IP + "\n"
		}
	}
	return ret
}

func hostsJSON() string {
	ret := "{"
	for _, bot := range robots {
		ret += "\"" + bot.Name + "\":"
		ret += "\"" + bot.IP + "\","
	}
	return strings.TrimSuffix(ret, ",") + "}"
}

func hostsAliveJSON() string {
	ret := "{"
	for _, bot := range robots {
		if bot.Alive == "true" {
			ret += "\"" + bot.Name + "\":"
			ret += "\"" + bot.IP + "\","
		}
	}
	return strings.TrimSuffix(ret, ",") + "}"
}

func textFull() string {
	ret := ""
	for _, bot := range robots {
		ret += bot.String() + "\n"
	}
	return ret
}

func demo() string {
	return `[{"Name":"leela","User":"legolas","IP":"10.147.158.134","X":"0.0080970386","Y":"-0.0129547603","Alive":"true","LastAlive":"1481563792"},
		{"Name":"bender","User":"jivko","IP":"10.147.158.135","X":"0.4080970386","Y":"-0.2129547603","Alive":"true","LastAlive":"1481563792"}
	]`
}

///////////////////////
// utility functions //
///////////////////////

func pdebug(message string) {
	if debug {
		fmt.Println(message)
	}
}

//error helper, prints error message if there's an error
func checkErr(err error, message string) bool {
	if err != nil {
		log.Printf("Error: %s\n", message)
		log.Println(err)
		return true
	}
	return false
}

/////////
// I/O //
/////////

func save() {
	currentTime := time.Now().Unix()
	if currentTime != timeLastSaved { //doesn't save more than once in 1 second
		pdebug("saving to local file")
		bytes, err := json.Marshal(robots)
		checkErr(err, "couldn't marshal the DNS")
		err = ioutil.WriteFile(file, bytes, 0644)
		checkErr(err, "couldn't write to "+file)
	}
	timeLastSaved = currentTime
}

func load() {
	pdebug("reading from " + file)
	bytes, err := ioutil.ReadFile(file)
	checkErr(err, "couldn't read from "+file)
	err = json.Unmarshal(bytes, &robots)
	checkErr(err, "couldn't unmarshal data read from "+file)
}

func updateAlive() {
	for {
		for i := range robots {
			robotTime, err := strconv.ParseInt(robots[i].LastAlive, 10, 64)
			if checkErr(err, "couldn't parse LastAlive in updateAlive()") {
				robots[i].Alive = "false"
			} else {
				robots[i].Alive = strconv.FormatBool(time.Now().Unix()-robotTime < aliveTimeout)

			}
		}
		pdebug("updated alive")
		<-aliveTicker.C
	}
}

func addRobot(query url.Values, addr string) {
	bot := Robot{
		Name:      query.Get("name"),
		User:      query.Get("user"),
		IP:        addr,
		X:         query.Get("x"),
		Y:         query.Get("y"),
		Level:     query.Get("level"),
		Alive:     "true",
		LastAlive: strconv.FormatInt(time.Now().Unix(), 10)}
	pdebug("Adding new robot: " + string(bot.Name))
	robots = append(robots, bot)
}

func updateRobot(query url.Values, addr string) (string, bool) {
	//saves the value of token then removes it so that the url string is identical
	//to the string hashed by the client
	token := query.Get("token")
	query.Del("token")

	robotTime, err := strconv.ParseInt(query.Get("timestamp"), 10, 64)
	checkErr(err, "couldn't parse timestamp from request")

	if !insecure {
		if !checkValidity(token, query.Encode()) { //query.Encode reorders (alphabetically)
			pdebug("invalid token from " + query.Get("name"))
			return "invalid token", false
		} else if math.Abs(time.Now().Unix()-robotTime) > tokenTimeout {

			pdebug("expired token from " + query.Get("name"))
			pdebug("system time: " + strconv.Itoa(int(time.Now().Unix())) + ", robot time: " + strconv.Itoa(int(robotTime)))

			return "expired token", false
		} else {
			pdebug("valid token from " + query.Get("name"))
		}
	}

	for i, bot := range robots {
		if bot.Name == query.Get("name") {
			robots[i].User = query.Get("user")
			robots[i].IP = addr
			if query.Get("x") != "" {
				robots[i].X = query.Get("x")
			}
			if query.Get("y") != "" {
				robots[i].Y = query.Get("y")
			}
			robots[i].LastAlive = strconv.FormatInt(time.Now().Unix(), 10)

			pdebug("Updated " + query.Get("name"))
			return "updated " + query.Get("name") + "\n\n" + " - - - - - -\n\n" + robots[i].String(), true
		}
	}
	addRobot(query, addr) // if it's not in robots, add it
	return "Added " + query.Get("name"), true
}

// check the validity of the message
func checkValidity(check string, stringURL string) bool {
	if password == "" {
		pdebug("reading from .cmasskey")
		path, err := filepath.Abs("./.cmasskey")
		checkErr(err, "couldn't create abs path from relative")
		bytes, err := ioutil.ReadFile(path)
		checkErr(err, "couldn't read from .cmasskey located at "+path)
		password = strings.TrimSpace(string(bytes[:])) // TrimSpace removes trailing \n or \r
	}

	pdebug("url: " + stringURL)
	pdebug("password: " + password)
	pdebug("hash: " + hash(stringURL, password))

	return check == hash(stringURL, password)
}

func hash(msg string, salt string) string {
	for i := 0; i < hashIterations; i++ {
		hasher.Reset()
		hasher.Write([]byte(msg + salt))
		msg = hex.EncodeToString(hasher.Sum(nil))
	}
	return msg
}

func main() {
	flag.BoolVar(&insecure, "insecure", false, "don't require token to authenticate robot updates")
	flag.BoolVar(&debug, "debug", false, "print debug info")
	flag.IntVar(&portNumber, "port", 7978, "port number")
	flag.StringVar(&file, "file", ".robot_statuses", "file to save robot statuses")
	flag.Parse()

	//load from local files/database
	load()

	//starts a new thread to periodically update who is 'alive'
	go updateAlive()

	//bind server
	http.HandleFunc("/update", update)
	http.HandleFunc("/json", serveBasicHTML(jsonFull))
	http.HandleFunc("/text", serveBasicHTML(textFull))
	http.HandleFunc("/hosts", serveBasicHTML(hosts))
	http.HandleFunc("/hostsalive", serveBasicHTML(hostsalive))
	http.HandleFunc("/hostsjson", serveBasicHTML(hostsJSON))
	http.HandleFunc("/hostsalivejson", serveBasicHTML(hostsAliveJSON))
	http.HandleFunc("/demo", serveBasicHTML(demo))

	fmt.Println("starting server on port " + strconv.Itoa(portNumber))
	log.Fatal(http.ListenAndServe(":"+strconv.Itoa(portNumber), nil))
}
