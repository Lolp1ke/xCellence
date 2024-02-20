# Whoever you are don't you dare say something about this repo

Everything separated into mini modules, so it should not be hard to understand what is what.
First thing first install some kind of markdown previewer plugin (optional).
I strongly recommend you to use android studio instead of general Rev Hardware Client.
If you do not understand this codebase it does not mean that code is shit, it means you have skill
issues.
Also I hope you like math you will need it a bit.
Before reading code and doing your stuff check FTC CenterStage season task and consider this seasons
game goals.
I may not to end repos documentation part because I am very lazy. But I do give a shit so I will end
it eventually, *long pause* ...someday :)
Good luck have fun!

### Things to know

1) Git basics
2) OOP (constructors, inheritance and overloading)
3) USE ONLY ONE STYLE OF WRITING CODE
    - naming variables
        - double horsePower = 0d; // camel case
        - double horse_power = 1d; // snake case
4) Follow folder structure
5) SEPARATE YOUR MODULES, NEVER WRITE WHOLE THING IN ONE FILE

### Movement

There are 2 types of driving available in this repo.
First one is 2wd, next one is 4wd.

##### 2wd

Code is very simple just read comments if I even wrote them. I may not, because code is very stupid
friendly.

##### 4wd

Firstly, I have no idea does this code works or not because we have no money to buy mecanum drive
kit for 4wd.
But in theory code have to work, if you relly using 4wd drive mode right now you can count yourself
as a very lucky person.
Read comments in file main/movement4wd.java

- Updated: btw code works, just use it and be happy ;)

### Mechanism

### Autonomous

What the ? HSV is?
HSV is one of the many color ranges like RGB or RGBA.
![img.png](https://th.bing.com/th/id/OIP.Jo4grR3nNkNXlovu5-iWAgHaHa?rs=1&pid=ImgDetMain)
First and last reason you must use HSV instead RGB is you just have to. HSV does not depends on
source of light as much as RGB does.

### Stuff

As you may notice code is separated into a modules and every module is responsible only for
specific part
Read main files comments

- Folders explained
    - main - the main script that will be used during competition (drive period)
      autonomous - the main script that will be used during competition (autonomous period)

    - sigma - new version of main that still in testing
      gamma - new version of autonomous that still in testing

    - alpha - old version of main code
      omega - old version of autonomous code

    - beta - folder for kids for learning and teaching them in future

### Why??

I guess you want me ask a lot of question so I would try to guess them and answer here

1) What the fuck "this" is? "this" refers to an attribute of a class.
   (Return back to Things to know and learn OOP)

```java
public class example {
	boolean attribute;


	public example(boolean attribute) {
		this.attribute = attribute; // "this.attribute" is attribute of the class "example" while "attribute" is argument of the constructor "example"
	}

	public example() {
		this(true); // "this" can call an constructor of the class and you can assign default values for attributes
	}
}
```
