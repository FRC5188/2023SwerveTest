package frc.lib.output;

/**
 * Utility class that's used for printing out statements to the console. 
 */
public abstract class Output {
    public static void print(String outputMessage)
    {   
        System.out.println("[INFO]: " + outputMessage);
    }

    public static void warning(String outputMessage)
    {
        System.out.println("[WARNING]: " + outputMessage);
    }

    public static void error(String outputMessage)
    {
        System.out.println("[ERROR]: " + outputMessage);
    }
}
