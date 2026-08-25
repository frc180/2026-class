public class Main {
    
    // I am a comment, I do not affect the code at all!
    public static void main(String[] args) {
        int result = doMath();
        System.out.println(result);
    }

    // Function called "say", returns nothing
    public static void say(String thingToPrint) {
        System.out.println(thingToPrint);
    }

    public static void doSomethingMultipleTimes(int howManyTimes, Runnable something) {
        for (int i = 0; i < howManyTimes; i++) {
           something.run();
        }
    }

    public static void printArray(String[] array) {
        for (String element : array) {
            System.out.println(element);
        }
    }

    public static int doMath() {
        return 4 + 5;
    }
}
