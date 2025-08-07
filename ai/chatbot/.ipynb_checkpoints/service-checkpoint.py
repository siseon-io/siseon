# chatbot/service.py
import json
from collections import Counter
from langchain_core.prompts import PromptTemplate
from langchain.chains import LLMChain
from langchain.memory import ConversationBufferMemory

from llm import GMSChat       # 절대 임포트
from indexer import retriever  # 절대 임포트

few_shot = """
Example 1:
Context: The gaze model runs in 65ms/frame ...
Answer: ...

Example 2:
...
Now:
"""

rewrite_prompt = PromptTemplate(
    input_variables=["chat_history", "question"],
    template="""
You are the SISEON Manual Assistant.
Conversation:
{chat_history}

Follow-up question: {question}

Standalone question:"""
)

answer_prompt_default = PromptTemplate(
    input_variables=["context", "question"],
    template=f"""
You are the SISEON Manual Expert.
Context:
{{context}}

Question:
{{question}}

{few_shot}
Answer in JSON with 'summary' and 'details':"""
)

answer_prompt_code = PromptTemplate(
    input_variables=["context", "question"],
    template="""
You are the SISEON Developer Assistant.
Context:
{context}

Question:
{question}

Answer with code in Markdown:
```python
# implementation...
```"""
)

# LLM + chains 초기화
llm = GMSChat()
memory = ConversationBufferMemory(memory_key="chat_history", return_messages=True)
rewriter_chain   = LLMChain(llm=llm, prompt=rewrite_prompt, verbose=False, memory=memory)
qa_chain_default = LLMChain(llm=llm, prompt=answer_prompt_default, verbose=False)
qa_chain_code    = LLMChain(llm=llm, prompt=answer_prompt_code, verbose=False)

def answer(question: str) -> dict:
    # 1) Rewrite to standalone question
    hist = memory.load_memory_variables({}).get("chat_history", [])
    hist_str = "\n".join(f"{m.role}: {m.content}" for m in hist)
    standalone_q = rewriter_chain.predict(chat_history=hist_str, question=question)

    # 2) RAG retrieve
    docs = retriever.get_relevant_documents(standalone_q)
    context = "\n\n".join(d.page_content for d in docs[:3])

    # 3) LLM call
    if standalone_q.lower().startswith("코드"):
        raw = qa_chain_code.predict(context=context, question=standalone_q)
    else:
        candidates = [
            qa_chain_default.predict(context=context, question=standalone_q)
            for _ in range(3)
        ]
        raw = Counter(candidates).most_common(1)[0][0]

    # 4) JSON parse & normalize
    try:
        res = json.loads(raw)
    except json.JSONDecodeError:
        # JSON이 아닌 경우, summary에 원본 텍스트, details는 빈 dict
        return {"summary": raw, "details": {}}

    summary = res.get("summary", "")
    details = res.get("details", {})

    # details가 문자열이라면 dict로 감싸기
    if not isinstance(details, dict):
        details = {"response": details}

    return {
        "summary": summary,
        "details": details
    }
