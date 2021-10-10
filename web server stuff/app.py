from flask import *
import rospy

# create the application object; tells Flask where to look for 
# resources such as templates and resource files
app = Flask(__name__)
        # __name__ is a shortcut to represent the application's module/package

# use decorators to link the function to a url 
# tells Flask what URL should trigger our function
@app.route('/')
def home():
    return "hello, world!" # return a string

@app.route('/welcome')
def welcome():
    return render_template('welcome.html') # render a template

@app.route('/index')
def index():
    return render_template('index.html')

@app.route('/robot_display')
def robot_display():
    return render_template('robot_display.html')

@app.route("/static/<path:path>")
def static_dir(path):
    try:
        #print("finding path")
        return send_from_directory("/static", path)
    except FileNotFoundError:
        #print("/static"+path+" not found")
        abort(404)

# start the server with the 'run()' method
if __name__ == "__main__":  
        # is guard; prevents importing file from accidentally triggering code below. 
        # i.e. code below only executes if app.py is the main program!
    app.run(port=2000, debug=True)


