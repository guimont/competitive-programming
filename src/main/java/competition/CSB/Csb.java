package competition.CSB;



import competitive.programming.genetic.GeneticAlgorithm;
import competitive.programming.genetic.GeneticAlgorithmTest;

import java.util.*;


/**
 * Created by gmo on 07/06/2017.
 */
public class Csb {


    static class Point {
        double x;
        double y;

        public Point(double x, double y) {
            this.x = x;
            this.y = y;
        }

        public Point() {
        }


        double distance2(Point p) {
            return (this.x - p.x)*(this.x - p.x) + (this.y - p.y)*(this.y - p.y);
        }

        double distance(Point p) {
            return Math.sqrt(this.distance2(p));
        }

        public Point closest (Point a, Point b)
        {
            double da = b.y - a.y;
            double db = a.x - b.x;
            double c1 = da*a.x + db*a.y;
            double c2 = -db*this.x + da*this.y;
            double det = da*da + db*db;
            double cx = 0;
            double cy = 0;

            if (det != 0) {
                cx = (da*c1 - db*c2) / det;
                cy = (da*c2 + db*c1) / det;
            } else {
                // The point is already on the line
                cx = this.x;
                cy = this.y;
            }

            return new Point(cx, cy);
        }


    }

    static class Collision {
        Unit a;
        Unit b;
        double t;

        public Collision(Unit a, Unit b, double t) {
            this.a = a;
            this.b = b;
            this.t = t;
        }

    }

    static abstract class Unit extends Point {

        int id;
        double vx;
        double vy;
        double r;

        public Unit(int x, int y) {
            super(x,y);
        }


        abstract void bounce(Unit u);


        Collision collision(Unit u) {
            // Square of the distance
            double dist = this.distance2(u);

            // Sum of the radii squared
            double sr = 357604; //(this.r + u.r)*(this.r + u.r);

            // We take everything squared to avoid calling sqrt uselessly. It is better for performances

            if (dist < sr) {
                // Objects are already touching each other. We have an immediate collision.
                return new Collision(this, u, 0.0);
            }

            // Optimisation. Objects with the same speed will never collide
            if (this.vx == u.vx && this.vy == u.vy) {
                return null;
            }

            // We place ourselves in the reference frame of u. u is therefore stationary and is at (0,0)
            double x = this.x - u.x;
            double y = this.y - u.y;
            Point myp = new Point(x, y);
            double vx = this.vx - u.vx;
            double vy = this.vy - u.vy;
            Point up = new Point(0, 0);

            // We look for the closest point to u (which is in (0,0)) on the line described by our speed vector
            Point p = up.closest(myp, new Point(x + vx, y + vy));

            // Square of the distance between u and the closest point to u on the line described by our speed vector
            double pdist = up.distance2(p);

            // Square of the distance between us and that point
            double mypdist = myp.distance2(p);

            // If the distance between u and this line is less than the sum of the radii, there might be a collision
            if (pdist < sr) {
                // Our speed on the line
                double length = Math.sqrt(vx * vx + vy * vy);

                // We move along the line to find the point of impact
                double backdist = Math.sqrt(sr - pdist);
                p.x = p.x - backdist * (vx / length);
                p.y = p.y - backdist * (vy / length);

                // If the point is now further away it means we are not going the right way, therefore the collision won't happen
                if (myp.distance2(p) > mypdist) {
                    return null;
                }

                pdist = p.distance(myp);

                // The point of impact is further than what we can travel in one turn
                if (pdist > length) {
                    return null;
                }

                // Time needed to reach the impact point
                double t = pdist / length;

                return new Collision(this, u, t);
            }

            return null;
        }


    }


    static class Checkpoint extends Unit {
        public Checkpoint(int x, int y) {
            super(x,y);
        }

        public Checkpoint(int x, int y, int i) {
            super(x,y);
        }

        void bounce(Unit u) {

        }


    }


    static class PodS extends Unit {
        double angle;
        int nextCheckpointId;
        int checked = 0;
        int timeout = 0;
        int thrust = 0;
        Pod partner;
        boolean shield;

        double breakDist;

        public PID pid = new PID();
        public int nbCP;

        public PodS(int x, int y) {
            super(x, y);
        }


        double getAngle(Point p) {
            double d = this.distance(p);
            double dx = (p.x - this.x) / d;
            double dy = (p.y - this.y) / d;

            // Simple trigonometry. We multiply by 180.0 / PI to convert radiants to degrees.
            double a = (Math.acos(dx) * 180.0 / Math.PI);

            // If the point I want is below me, I have to shift the angle for it to be correct
            if (dy < 0) {
                a = 360.0 - a;
            }

            return a;
        }


        double diffAngle(Point p) {
            double a = this.getAngle(p);

            // To know whether we should turn clockwise or not we look at the two ways and keep the smallest
            // The ternary operators replace the use of a modulo operator which would be slower
            double right = this.angle <= a ? a - this.angle : 360.0 - this.angle + a;
            double left = this.angle >= a ? this.angle - a : this.angle + 360.0 - a;

            if (right < left) {
                return right;
            } else {
                // We return a negative angle if we must rotate to left
                return -left;
            }
        }


        void rotate(Point p) {
            double a = this.diffAngle(p);

            // Can't turn by more than 18° in one turn
            if (a > 18.0) {
                a = 18.0;
            } else if (a < -18.0) {
                a = -18.0;
            }

            this.angle += a;

            // The % operator is slow. If we can avoid it, it's better.
            if (this.angle >= 360.0) {
                this.angle = this.angle - 360.0;
            } else if (this.angle < 0.0) {
                this.angle += 360.0;
            }
        }

        public int calThrust(Point p, Point pNext) {

            double angle = 0;

            if (this.distance(p) > breakDist)
                angle =  this.diffAngle(p);
            else
                angle = this.diffAngle(pNext) ;


            return (int)pid.processing(angle);
        }


        void boost(int thrust) {
            // Don't forget that a pod which has activated its shield cannot accelerate for 3 turns
            if (this.shield) {
                return;
            }

            // Conversion of the angle to radiants
            double ra = this.angle * Math.PI / 180.0;

            // Trigonometry
            this.vx += Math.cos(ra) * thrust;
            this.vy += Math.sin(ra) * thrust;

            this.thrust = thrust;
        }

        void move(double t) {
            this.x += this.vx * t;
            this.y += this.vy * t;
        }


        void end() {
            this.x = Math.round(this.x);
            this.y = Math.round(this.y);
            this.vx = Math.floor(this.vx * 0.85);
            this.vy = Math.floor(this.vy * 0.85);

            // Don't forget that the timeout goes down by 1 each turn. It is reset to 100 when you pass a checkpoint
            this.timeout -= 1;
        }

        void play(Point p, Point pNext) {
            this.rotate(p);
            this.boost(calThrust(p,pNext));
        }

        void play(PodS[] pods, List<Checkpoint> checkpoints) {
            // This tracks the time during the turn. The goal is to reach 1.0
            double t = 0.0;

            while (t < 1.0) {
                Collision firstCollision = null;

                // We look for all the collisions that are going to occur during the turn
                for (int i = 0; i < pods.length; ++i) {
                    // Collision with another pod?
                    for (int j = i + 1; j < pods.length; ++j) {
                        Collision col = pods[i].collision(pods[j]);

                        // If the collision occurs earlier than the one we currently have we keep it
                        if (col != null && col.t + t < 1.0 && (firstCollision == null || col.t < firstCollision.t)) {
                            firstCollision = col;
                        }
                    }

                    // Collision with another checkpoint?
                    // It is unnecessary to check all checkpoints here. We only test the pod's next checkpoint.
                    // We could look for the collisions of the pod with all the checkpoints, but if such a collision happens it wouldn't impact the game in any way
                    Collision col = pods[i].collision(checkpoints.get(pods[i].nextCheckpointId));

                    // If the collision happens earlier than the current one we keep it
                    if (col != null && col.t + t < 1.0 && (firstCollision == null || col.t < firstCollision.t)) {
                        firstCollision = col;
                    }
                }

                if (firstCollision == null) {
                    // No collision, we can move the pods until the end of the turn
                    for (int i = 0; i < pods.length; ++i) {
                        pods[i].move(1.0 - t);
                    }

                    // End of the turn
                    t = 1.0;
                } else {
                    // Move the pods to reach the time `t` of the collision
                    for (int i = 0; i < pods.length; ++i) {
                        pods[i].move(firstCollision.t);
                    }

                    // Play out the collision
                    firstCollision.a.bounce(firstCollision.b);

                    t += firstCollision.t;
                }
            }

            for (int i = 0; i < pods.length; ++i) {
                pods[i].end();
            }
        }


        void bounce(PodS u) {
            {
                // If a pod has its shield active its mass is 10 otherwise it's 1
                double m1 = this.shield ? 10 : 1;
                double m2 = u.shield ? 10 : 1;
                double mcoeff = (m1 + m2) / (m1 * m2);

                double nx = this.x - u.x;
                double ny = this.y - u.y;

                // Square of the distance between the 2 pods. This value could be hardcoded because it is always 800²
                double nxnysquare = nx*nx + ny*ny;

                double dvx = this.vx - u.vx;
                double dvy = this.vy - u.vy;

                // fx and fy are the components of the impact vector. product is just there for optimisation purposes
                double product = nx*dvx + ny*dvy;
                double fx = (nx * product) / (nxnysquare * mcoeff);
                double fy = (ny * product) / (nxnysquare * mcoeff);

                // We apply the impact vector once
                this.vx -= fx / m1;
                this.vy -= fy / m1;
                u.vx += fx / m2;
                u.vy += fy / m2;

                // If the norm of the impact vector is less than 120, we normalize it to 120
                double impulse = (double)Math.sqrt(fx*fx + fy*fy);
                if (impulse < 120.0) {
                    fx = fx * 120.0 / impulse;
                    fy = fy * 120.0 / impulse;
                }

                // We apply the impact vector a second time
                this.vx -= fx / m1;
                this.vy -= fy / m1;
                u.vx += fx / m2;
                u.vy += fy / m2;

                // This is one of the rare places where a Vector class would have made the code more readable.
                // But this place is called so often that I can't pay a performance price to make it more readable.
            }
        }


        @Override
        void bounce(Unit u) {
            if (u instanceof Checkpoint) {
                checked += 1;
                timeout = 100;
                nextCheckpointId = (nextCheckpointId+1) % nbCP;
                //ncpid = (ncpid + 1) % cp_ct;
                return;
            }

            bounce((PodS)u);

        }
    }


    /*
    nextCheckpointAngle: 7828
nextCheckpointAngle: 883
nextCheckpointAngle: 7648
nextCheckpointAngle: 5978
nextCheckpointAngle: 3165
nextCheckpointAngle: 7529
nextCheckpointAngle: 9537
nextCheckpointAngle: 4353
nextCheckpointAngle: 14495
nextCheckpointAngle: 7760
nextCheckpointAngle: 6349
nextCheckpointAngle: 4284
     */




    static class Simu {
        //
        List<Checkpoint> cpList = new ArrayList();
        PodS pod[] = {new PodS(3859,6991)};

        void init() {
            cpList.add(new Checkpoint(13506, 2347));
            cpList.add(new Checkpoint(12938, 7229));
            cpList.add(new Checkpoint(5617, 2559));
            cpList.add(new Checkpoint(4097, 7431));
            /*cpList.add(new Checkpoint(14495, 7760));
            cpList.add(new Checkpoint(6349, 4284));*/
            pod[myPod].angle = pod[myPod].diffAngle(cpList.get(0));
            pod[myPod].nbCP = cpList.size();
        }



        int loop = 0;
        void start() {
            while(pod[myPod].checked < 12) {
                Checkpoint cp = cpList.get(pod[myPod].nextCheckpointId);
                pod[0].play(cp, pod[myPod].nextCheckpointId < 3 ? cpList.get(pod[myPod].nextCheckpointId+1) : cp);
                pod[0].play(pod, cpList);
                loop ++;

                //failled
                if (pod[myPod].timeout == 0)  {
                    loop = 1000;
                    return;
                }
            }

            System.err.println("loop: " + loop);
        }

        /*cpList.add(new Checkpoint(7828, 883));
        Checkpoint cp = new Checkpoint(7648, 5978);*/
    }


    private static final Random random = new Random(0);

    private static class Combination {
        public static Csb.Combination newInstance() {
            generatorValue = (generatorValue + 0.1) % 1;
            return new Csb.Combination(generatorValue, generatorValue, generatorValue, generatorValue*3000);
        }

        double Kp = 0.15;
        double Ki = 0.1;
        double Kd = 0;
        double breakDist;
        private static double generatorValue = 0;

        public Combination(double kp, double ki, double kd, double breakDist) {
            Kp = kp;
            Ki = ki;
            Kd = kd;
            this.breakDist = breakDist;
        }

        public double evaluate() {

            Simu sim = new Simu();
            sim.init();
            sim.pod[myPod].pid.init(Kp, Ki,Kd);
            sim.pod[myPod].breakDist = breakDist;
            sim.start();


            //inverse score
            return 1000 - sim.loop;
        }



        public Csb.Combination merge(Csb.Combination other) {
            return new Csb.Combination(randomBoolean() ? Kp : other.Kp, randomBoolean() ? Ki : other.Ki, randomBoolean() ? Kd : other.Kd,
                    randomBoolean() ? breakDist : other.breakDist);
        }

        public Csb.Combination mutate() {
            return new Csb.Combination(Kp*0.8, Ki*0.8, Kd*0.8, breakDist + 100);
        }

        private boolean randomBoolean() {
            return random.nextBoolean();
        }

        @Override
        public String toString() {
            return "Combination{" +
                    "Kp=" + Kp +
                    ", Ki=" + Ki +
                    ", Kd=" + Kd +
                    ", breakDist=" + breakDist +
                    '}';
        }
    }


    static class Optimize {

        void run() {

            final GeneticAlgorithm<Csb.Combination> algo = new GeneticAlgorithm<Csb.Combination>(c -> c.evaluate(), () -> Csb.Combination.newInstance(),
                    (first, second) -> first.merge(second), c -> c.mutate());


    /*
            algo.setShuffler(list -> {
                Collections.shuffle(list, random);
            });
*/
            algo.initialize(10);
            algo.iterate(1000, 15, 20, 20, 20);
            //algo.printTo(System.err);
            System.err.println(algo.best().toString());


            algo.best().evaluate();
        }

    }


    /*
    Tous les x millisecondes, faire :
    erreur = consigne - mesure;
    somme_erreurs += erreur;
    variation_erreur = erreur - erreur_précédente;
    commande = Kp * erreur + Ki * somme_erreurs + Kd * variation_erreur;
    erreur_précédente = erreur
    */

    /*
    L'erreur statique, c'est l'erreur finale une fois que le système est stabilité. Cette erreur doit être nulle. Pour diminuer l'erreur statique, il faut augmenter Kp et Ki.
Le dépassement, c'est le rapport entre le premier pic et la consigne. Ce dépassement diminue si Kp ou Ki diminuent ou si Kd augmente.
Le temps de montée correspond au temps qu'il faut pour arriver ou dépasser à la consigne. Le temps de montée diminue si Kp ou Ki augmentent ou si Kd diminue.
Le temps de stabilisation, c'est le temps qu'il faut pour que le signal commette une erreur inférieure à 5% de la consigne. Ce temps de stabilisation diminue quand Kp et Ki augmentent.
*/


    static double ANGLE_NO_DEFINE = 1000;


    /**
     * Check Point
     */
    static class CP {
        int posx = 0;
        int posy = 0;
        int posCP = 0;
        double angle = ANGLE_NO_DEFINE;

        public CP(int posx, int posy, int posCP) {
            this.posx = posx;
            this.posy = posy;
            this.posCP = posCP;
        }


    }


    /**
     * Pod
     */
    static class Pod {
        int startPosX = 0;
        int startPosY = 0;
        int posx = 0;
        int posy = 0;
        int posCP = -1;
        double currentAngle = ANGLE_NO_DEFINE;
        int thrust = 0;
        boolean boost = true;
        int shield = 0;
        public boolean newCP;
        public PID pid = new PID();
        public int distanceCP;
        int loop = 1;
        public int largeLine = -1;

        public Pod(int posx, int posy, int posCP) {
            this.posx = posx;
            this.posy = posy;
            this.posCP = posCP;
        }

        public int calThrust(double angle) {
            if (loop > 1 && distanceCP < 2000 && thrust > 50)
                thrust = (int)pid.processing(angle);

            else thrust = (int)pid.processing(currentAngle);
            return thrust;
        }

        public int getThrust() {
            return thrust;
        }


        public boolean update(int x, int y, CP cp, int nextCheckpointDist, int nextCheckpointAngle) {
            boolean trackFull = false;
            this.posx = x;
            this.posy = y;

            if (this.posCP != cp.posCP) {
                this.newCP = true;
                pid.reset();
                if (cp.posCP == 0) {
                    loop ++;
                    if (loop == 2) trackFull = true;
                }
            }
            else
                this.newCP = false;



            System.err.println("loop: " + loop);
            this.posCP = cp.posCP;

            this.currentAngle = nextCheckpointAngle;
            this.distanceCP = nextCheckpointDist;

            return trackFull;
        }


    }


    /**
     * PID
     */
    static int order = 0;
    static class PID {
        double error = 0;
        double errorLast = 0;
        double errorSum = 0;
        double variation = 0;

        double Kp = 0.15;
        double Ki = 0.1;
        double Kd = 0;


        double processing( double angle) {
            error = order - angle;

            variation = error - errorLast;
            double thrust = Kp * error + Ki * errorSum + Kd*variation;
            errorLast = error;
            errorSum += error;

            //return 100;
            if (Math.abs(error) < 5) return 100;
            return Math.max(0,Math.min(100,100 -Math.abs(thrust)));
        }


        public void reset() {
            errorSum = 0;
            errorLast = 0;
        }

        public void init(double kp, double ki, double kd) {
            Kp = kp;
            Ki = ki;
            Kd = kd;
        }
    }


    static int myPod = 0;
    static int challengerPod = 1;
    static class Game {
        List<CP> cpList = new ArrayList();
        Pod pod[] = {new Pod(0,0,0),new Pod(0,0,0)};


        void update(int x,int y,int nextCheckpointX,int nextCheckpointY ,int nextCheckpointDist,int nextCheckpointAngle,int opponentX,int opponentY) {
            //check cp
            CP cp = updateCP(nextCheckpointX,nextCheckpointY);
            if (pod[myPod].update(x, y, cp, nextCheckpointDist, nextCheckpointAngle)) {
                calAngle();
                cpList.forEach(c -> {
                    System.err.println("nextCheckpointAngle: " + c.posx);
                    System.err.println("nextCheckpointAngle: " + c.posy);
                });

                System.err.println("start: " + pod[myPod].largeLine);
                System.err.println("start: " + pod[myPod].startPosX);
                System.err.println("start: " + pod[myPod].startPosY);

            }
            pod[challengerPod].update(opponentX,opponentY,cp,-1,-1);
        }


        CP updateCP(int x, int y) {
            for (CP cp : cpList) {
                if (cp.posx == x && cp.posy == y)
                    return cp;
            }

            CP cp  = new CP(x,y,cpList.size());

            cpList.add(cp);
            return cp;
        }

        String getAction() {

            int cpPos = pod[myPod].posCP;

            if (pod[myPod].largeLine == cpPos && pod[myPod].boost && pod[myPod].currentAngle < 5) {
                pod[myPod].boost = false;
                return  cpList.get(cpPos).posx + " " + cpList.get(cpPos).posy + " " + "BOOST" + " " + "BOOST";

            }

            return  cpList.get(cpPos).posx + " " + cpList.get(cpPos).posy + " " + pod[myPod].calThrust(cpList.get(cpPos).angle) + " " + pod[myPod].getThrust();
        }

        public void startPos(int x, int y) {
            pod[myPod].startPosX = x;
            pod[myPod].startPosY = y;
        }

        public void calAngle() {

            int s = cpList.size();

            for (int i = 0; i < cpList.size(); i++) {

                int x1 = cpList.get(i % s).posx;
                int y1 = cpList.get(i % s).posy;

                int x2 = cpList.get((i+1) % s).posx;
                int y2 = cpList.get((i+1) % s).posy;

                int x3 = cpList.get((i+2) % s).posx;
                int y3 = cpList.get((i+2) % s).posy;

                double a = distance(x1, y1, x2, y2);
                double b = distance(x2, y2, x3, y3);
                double c = distance(x3, y3, x1, y1);

                double coef = Math.pow(a, 2) + Math.pow(b, 2) - Math.pow(c, 2);
                double denom = (2 * a * b);
                double angle = Math.acos(coef / denom);
                angle = 180 - (180 * (angle) / Math.PI);
                System.err.println("angle: " + angle);
                cpList.get((i+1) % s).angle = angle;
            }

            double max = 0;
            int node = 0;
            for (int i = 0; i < cpList.size()-1; i++) {
                int x1 = cpList.get(i).posx;
                int y1 = cpList.get(i).posy;

                int x2 = cpList.get((i+1)).posx;
                int y2 = cpList.get((i+1)).posy;
                double a = distance(x1, y1, x2, y2);
                if (a > max) node = i;
                max = Math.max(max,a);
            }

            pod[myPod].largeLine = node;


        }

    }


    static public double distance (int x1, int y1, int x2, int y2)
    {
        return Math.sqrt(Math.pow((x2-x1),2)+Math.pow((y2-y1),2));
    }


    /*public static void main(String args[]) {
        Game game = new Game();
        game.updateCP(1,1);
        game.updateCP(3,1);
        game.updateCP(3,3);

        game.calAngle();

    }*/


    public static void main(String args[]) {

        /*Simu sim = new Simu();
        sim.init();
        sim.start();*/

        Optimize op = new Optimize();
        op.run();

        Scanner in = new Scanner(System.in);

        int max = 0;

        boolean start = true;
        Game game = new Game();

        // game loop
        while (true) {
            int x = in.nextInt();
            int y = in.nextInt();
            int nextCheckpointX = in.nextInt(); // x position of the next check point
            int nextCheckpointY = in.nextInt(); // y position of the next check point
            int nextCheckpointDist = in.nextInt(); // distance to the next checkpoint
            int nextCheckpointAngle = in.nextInt(); // angle between your pod orientation and the direction of the next checkpoint
            int opponentX = in.nextInt();
            int opponentY = in.nextInt();

            if (start) {
                start = false;
                game.startPos(x,y);
            }


            game.update(x,y,nextCheckpointX,nextCheckpointY,nextCheckpointDist,nextCheckpointAngle,opponentX,opponentY);




            // Write an action using System.out.println()
            // To debug: System.err.println("Debug messages...");


            // You have to output the target position
            // followed by the power (0 <= thrust <= 100)
            // i.e.: "x y thrust"

            max = Math.max(nextCheckpointDist,max);
            System.err.println("nextCheckpointAngle: " + nextCheckpointAngle);
            System.err.println("nextCheckpointDist: " + nextCheckpointDist);
            System.err.println("distance: " + distance(x,y,opponentX,opponentY));




            System.out.println(game.getAction());
        }
    }
}
