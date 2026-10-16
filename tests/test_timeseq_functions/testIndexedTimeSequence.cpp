#include <QtTest/QtTest>

#include "vision/indexed_timed_sequence.h"

class TestIndexedTimeSequence: public QObject
{
    Q_OBJECT
private Q_SLOTS:
    void testIndexTimePerStep();
    void testGetValueAtTime();
    void testGetValuesCoveringTimes();
    void testGetValuesInBetweenTimes();
    void testGetValuesInBetweenTimesSymmetry();
    void testGetValuesForIntegration();
};

void TestIndexedTimeSequence::testIndexTimePerStep() {

    using TimedElement = StereoVisionApp::IndexedTimeSequence<double>::TimedElement;

    constexpr int nElements = 10;
    constexpr double dt = 0.4269;

    std::vector<TimedElement> data(nElements);

    for (int i = 0; i < nElements; i++) {
        data[i].time = i*dt;
        data[i].val = i;
    }

    StereoVisionApp::IndexedTimeSequence<double> sequence(data);

    QCOMPARE(sequence.indexTimePerStep(), dt);


}

void TestIndexedTimeSequence::testGetValueAtTime() {

    using TimedElement = StereoVisionApp::IndexedTimeSequence<double>::TimedElement;

    constexpr int nElements = 10;

    std::vector<TimedElement> data(nElements);

    for (int i = 0; i < nElements; i++) {
        data[i].time = i;
        data[i].val = i;
    }

    StereoVisionApp::IndexedTimeSequence<double> sequence(data);

    //check elements within the sequence
    for (int i = 0; i < nElements-1; i++) {

        std::array<double,5> dts = {0, 0.25, 0.5, 0.75};

        for (double dt : dts) {

            double time = i + dt;

            auto interpolable = sequence.getValueAtTime(time);

            double val = interpolable.weightLower*interpolable.valLower + interpolable.weightUpper*interpolable.valUpper;

            QCOMPARE(val, time);

        }

    }

    //check elements outside of the sequence (nearest interpolation is applied outside the sequence)
    std::array<double,5> dts = {0.25, 0.5, 0.75};

    for (double dt : dts) {

        double time0 = -dt;
        double timeN = nElements-1 + dt;

        auto interpolable0 = sequence.getValueAtTime(time0);
        auto interpolableN= sequence.getValueAtTime(timeN);

        double val0 = interpolable0.weightLower*interpolable0.valLower + interpolable0.weightUpper*interpolable0.valUpper;
        double valN = interpolableN.weightLower*interpolableN.valLower + interpolableN.weightUpper*interpolableN.valUpper;

        QCOMPARE(val0, 0);
        QCOMPARE(valN, nElements-1);
    }

}

void TestIndexedTimeSequence::testGetValuesCoveringTimes() {

    using TimedElement = StereoVisionApp::IndexedTimeSequence<double>::TimedElement;

    std::vector<TimedElement> data = {{.time = 0., .val = 0.},
                                      {.time = 1.3, .val = 0.},
                                      {.time = 2., .val = 0.},
                                      {.time = 3., .val = 10.},
                                      {.time = 4., .val = 10.},
                                      {.time = 5.5, .val = 15.},
                                      {.time = 6., .val = 11.},
                                      {.time = 8., .val = 10.}};

    StereoVisionApp::IndexedTimeSequence<double> sequence(data);

    double t0;
    double tf;

    //forward direction

    t0 = -2.5;
    tf = 1.5;

    auto elements = sequence.getValuesCoveringTimes(t0,tf);

    QCOMPARE(elements.size(), 3);
    for (int i = 0; i < 3; i++) {
        QCOMPARE(elements[i].time, data[i].time);
        QCOMPARE(elements[i].val, data[i].val);
    }

    t0 = 1.7;
    tf = 4.1;

    elements = sequence.getValuesCoveringTimes(t0,tf);

    QCOMPARE(elements.size(), 5);
    for (int i = 0; i < 5; i++) {
        QCOMPARE(elements[i].time, data[i+1].time);
        QCOMPARE(elements[i].val, data[i+1].val);
    }


    t0 = 5.6;
    tf = 9;

    elements = sequence.getValuesCoveringTimes(t0,tf);

    QCOMPARE(elements.size(), 3);
    for (int i = 0; i < 3; i++) {
        QCOMPARE(elements[i].time, data[i+5].time);
        QCOMPARE(elements[i].val, data[i+5].val);
    }

    //backward direction


    t0 = 1.5;
    tf = -2.5;

    elements = sequence.getValuesCoveringTimes(t0,tf);

    QCOMPARE(elements.size(), 3);
    for (int i = 0; i < 3; i++) {
        QCOMPARE(elements[i].time, data[2-i].time);
        QCOMPARE(elements[i].val, data[2-i].val);
    }

    t0 = 5.3;
    tf = 1.9;

    elements = sequence.getValuesCoveringTimes(t0,tf);

    QCOMPARE(elements.size(), 5);
    for (int i = 0; i < 5; i++) {
        QCOMPARE(elements[i].time, data[5-i].time);
        QCOMPARE(elements[i].val, data[5-i].val);
    }

    t0 = 9;
    tf = 5.6;

    elements = sequence.getValuesCoveringTimes(t0,tf);

    QCOMPARE(elements.size(), 3);
    for (int i = 0; i < 3; i++) {
        QCOMPARE(elements[i].time, data[7-i].time);
        QCOMPARE(elements[i].val, data[7-i].val);
    }

}

void TestIndexedTimeSequence::testGetValuesInBetweenTimes() {

    using TimedElement = StereoVisionApp::IndexedTimeSequence<double>::TimedElement;

    constexpr int nElements = 10;

    std::vector<TimedElement> data(nElements);

    for (int i = 0; i < nElements; i++) {
        data[i].time = i;
        data[i].val = i;
    }

    StereoVisionApp::IndexedTimeSequence<double> sequence(data);

    double t0;
    double tf;
    double dt;

    //test get elements before the start of the sequence.
    t0 = -2.5;
    tf = 1.5;

    auto elements = sequence.getValuesInBetweenTimes(t0,tf);

    dt = 0;
    for (size_t i = 0; i < elements.size(); i++) {
        dt += elements[i].dt;
        QVERIFY(dt >= 0);
    }

    QCOMPARE(dt, tf-t0);

    //test get elements at the center of the sequence.
    t0 = 2.5;
    tf = 6.5;

    elements = sequence.getValuesInBetweenTimes(t0,tf);

    dt = 0;
    for (size_t i = 0; i < elements.size(); i++) {
        dt += elements[i].dt;
        QVERIFY(dt >= 0);
    }

    QCOMPARE(dt, tf-t0);

    //test get elements after the end of the sequence.
    t0 = 7.5;
    tf = 11.5;

    elements = sequence.getValuesInBetweenTimes(t0,tf);

    dt = 0;
    for (size_t i = 0; i < elements.size(); i++) {
        dt += elements[i].dt;
        QVERIFY(dt >= 0);
    }

    QCOMPARE(dt, tf-t0);

    //test get elements in reverse direction before the start of the sequence
    t0 = 2.5;
    tf = -1.5;

    elements = sequence.getValuesInBetweenTimes(t0,tf);

    dt = 0;
    for (size_t i = 0; i < elements.size(); i++) {
        dt += elements[i].dt;
        QVERIFY(dt <= 0);
    }

    QCOMPARE(dt, tf-t0);

    //test get elements in reverse direction at the center of the sequence.
    t0 = 4.5;
    tf = 1.5;

    elements = sequence.getValuesInBetweenTimes(t0,tf);

    dt = 0;
    for (size_t i = 0; i < elements.size(); i++) {
        dt += elements[i].dt;
        QVERIFY(dt <= 0);
    }

    QCOMPARE(dt, tf-t0);

    //test get elements in reverse direction after the end of the sequence.
    t0 = 12.5;
    tf = 8.5;

    elements = sequence.getValuesInBetweenTimes(t0,tf);

    dt = 0;
    for (size_t i = 0; i < elements.size(); i++) {
        dt += elements[i].dt;
        QVERIFY(dt <= 0);
    }

    QCOMPARE(dt, tf-t0);

}

void TestIndexedTimeSequence::testGetValuesInBetweenTimesSymmetry() {

    using TimedElement = StereoVisionApp::IndexedTimeSequence<double>::TimedElement;

    std::vector<TimedElement> data = {{.time = 0., .val = 0.},
                                      {.time = 1., .val = 0.},
                                      {.time = 2., .val = 0.},
                                      {.time = 3., .val = 10.},
                                      {.time = 4., .val = 10.}};

    double t0 = 1.00001;
    double tf = 2.99999;
    double dt;

    StereoVisionApp::IndexedTimeSequence<double> sequence(data);

    auto elements = sequence.getValuesInBetweenTimes(t0,tf);
    auto elementsRev = sequence.getValuesInBetweenTimes(tf,t0);

    dt = 0;
    for (size_t i = 0; i < elements.size(); i++) {
        dt += elements[i].dt;
        QVERIFY(dt >= 0);
    }

    QCOMPARE(dt, tf-t0);

    dt = 0;
    for (size_t i = 0; i < elementsRev.size(); i++) {
        dt += elementsRev[i].dt;
        QVERIFY(dt <= 0);
    }

    QCOMPARE(-dt, tf-t0);

    double sum = 0;
    double sumRev = 0;

    for (size_t i = 0; i < elements.size(); i++) {
        sum += elements[i].val*elements[i].dt;
    }

    for (size_t i = 0; i < elementsRev.size(); i++) {
        sumRev += elementsRev[i].val*elementsRev[i].dt;
    }

    double delta = sum + sumRev; //compute the closure assuming sum should approximately equal -sumRev;

    QVERIFY(std::abs(delta) < 1e-3);

}

void TestIndexedTimeSequence::testGetValuesForIntegration() {

    using TimedElement = StereoVisionApp::IndexedTimeSequence<double>::TimedElement;

    constexpr double val = 42.;

    std::vector<TimedElement> data = {{.time = 0., .val = val},
                                      {.time = 1., .val = val},
                                      {.time = 2., .val = val},
                                      {.time = 3., .val = val},
                                      {.time = 4., .val = val}};

    StereoVisionApp::IndexedTimeSequence<double> sequence(data);


    double t0 = 1.00001;
    double tf = 2.99999;
    double dt = tf-t0;

    double sum = 0;

    auto elements = sequence.getValuesInBetweenTimes(t0,tf);

    for (size_t i = 0; i < elements.size(); i++) {
        sum += elements[i].val*elements[i].dt;
    }

    QCOMPARE(sum, dt*val);


    t0 = 3.2;
    tf = 0.5;
    dt = tf-t0;

    sum = 0;

    elements = sequence.getValuesInBetweenTimes(t0,tf);

    for (size_t i = 0; i < elements.size(); i++) {
        sum += elements[i].val*elements[i].dt;
    }

    QCOMPARE(sum, dt*val);
}

QTEST_MAIN(TestIndexedTimeSequence)
#include "testIndexedTimeSequence.moc"
