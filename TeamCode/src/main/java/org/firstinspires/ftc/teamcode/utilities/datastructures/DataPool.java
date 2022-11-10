package org.firstinspires.ftc.teamcode.utilities.datastructures;

import org.firstinspires.ftc.teamcode.utilities.MathHelper;

import java.util.ArrayList;

class DataPoolNode <T> {

    T data;
    long nodeTimestamp;

    DataPoolNode (T data, long nodeTimestamp) {
        this.data = data;
        this.nodeTimestamp = nodeTimestamp;
    }

}

public class DataPool<T extends Number> {

    ArrayList<DataPoolNode<T>> pool;

    DataPool() {
        this.pool = new ArrayList<>();
    }

    public void addData(T data) {

        this.pool.add(
                new DataPoolNode<>(data, System.currentTimeMillis())
        );

    }

    public double getDataAtTimestamp(long timestamp) {

        int poolSize = this.pool.size();

        for (int i = 0; i < poolSize; i++) {

            DataPoolNode<T> currentIndexNode = this.pool.get(i);
            double currentData = (double) currentIndexNode.data;

            if (i == poolSize - 1) {
                return (double) currentIndexNode.data;
            }

            DataPoolNode<T> nextIndexNode = this.pool.get(i);
            double nextData = (double) nextIndexNode.data;

            if (timestamp < nextIndexNode.nodeTimestamp && timestamp > currentIndexNode.nodeTimestamp) {
                double dt = nextIndexNode.nodeTimestamp - currentIndexNode.nodeTimestamp;
                double d1 = timestamp - currentIndexNode.nodeTimestamp;
                double r = d1 / dt;

                return MathHelper.lerp(currentData, nextData, r);
            }
        }

        return 0;
     }

}
