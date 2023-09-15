class Test: #class는 항상 대문자
  def __init__(self): #self는 다른 문자로 하면 안된다. 항상 self라고 하여야 한다. 또한 __init__(self)가 없으면 ob = Test() 라고 표현이 안된다.
    self.var = 20 #class안에 변수를 선언하기 위해서는 'self.' 이라고 쓰고 뒤에 변수를 선언해야 한다.
    pass # 함수를 선언하고 향후 작성하기 위해 pass를 적어놓는다. 내부 내용이 아무것도 없으면 문제가 생기기 때문이다.

  def aa(self):
    print('aaaaaa')

import Test # #include와 동일(외부의 class를 불러온다.)

#class는 하나만 선언하여 내부에 모든 기능을 선언하는 것이 대표적이다.


class inh(Test): #Test class의 내용을 포함함
  def __init__(self):
    super().__init__() #super의 뜻은 부모 class(Test)를 참조하라


def main():
  a = 10 #중괄호를 사용하지 않기 때문에 띄어쓰기 중요.
  a = 'hello'
  ob = Test()
  inob = inh()

  ob.aa()
  ob.aa()

  print(a)

if __name__ == "__main__": #__  __는 내부변수라는 뜻
  main() #내부함수
# 메인함수 시작
