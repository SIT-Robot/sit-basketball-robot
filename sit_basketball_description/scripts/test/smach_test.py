import smach


class Foo(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['outcome1', 'outcome2']
        )
        self.counter = 0

    # override
    def execute(self, userdata):
        print('executing state foo')
        if self.counter < 3:
            self.counter += 1
            return 'outcome1'
        else:
            return 'outcome2'


class Bar(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['outcome2'])

    # override
    def execute(self, userdata):
        print('executing state bar')
        return 'outcome2'


if __name__ == '__main__':
    sm = smach.StateMachine(
        outcomes=['outcome4','outcome5']
    )
    with sm:
        smach.StateMachine.add(
            label='foo',
            state=Foo(),
            transitions={
                'outcome1': 'bar',
                'outcome2': 'outcome4',
            })

        smach.StateMachine.add(
            label='bar',
            state=Bar(),
            transitions={
                'outcome2': 'foo',
            }
        )

    outcome = sm.execute()