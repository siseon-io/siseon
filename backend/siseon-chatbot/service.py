# chatbot/service.py
import json
from collections import Counter
from langchain_core.prompts import ChatPromptTemplate, MessagesPlaceholder
from langchain_core.output_parsers import StrOutputParser
from langchain_core.runnables import RunnableWithMessageHistory
from langchain_community.chat_message_histories import ChatMessageHistory

from llm import GMSChat        # 그대로 사용
from indexer import retriever  # 그대로 사용

# ===== 세션별 메모리 저장소 =====
_memory_store: dict[str, ChatMessageHistory] = {}

def get_session_history(session_id: str) -> ChatMessageHistory:
    return _memory_store.setdefault(session_id, ChatMessageHistory())

# ===== 프롬프트 =====
few_shot = """
Example 1:
Context: The gaze model runs in 65ms/frame ...
Answer: ...

Example 2:
...
Now:
"""

rewrite_prompt = ChatPromptTemplate.from_messages([
    ("system", "You are the SISEON Manual Assistant."),
    MessagesPlaceholder("chat_history"),
    ("human", "Follow-up question: {question}\n\nStandalone question:")
])

answer_prompt_default = ChatPromptTemplate.from_messages([
    ("system", f"You are the SISEON Manual Expert.\n{few_shot}"),
    ("system", "Answer in JSON with 'summary' and 'details':"),
    ("system", "Context:\n{context}"),
    ("human", "Question:\n{question}")
])

answer_prompt_code = ChatPromptTemplate.from_messages([
    ("system", "You are the SISEON Developer Assistant."),
    ("system", "Answer with code in Markdown:\n```python\n# implementation...\n```"),
    ("system", "Context:\n{context}"),
    ("human", "Question:\n{question}")
])

# ===== LLM & 체인 =====
llm = GMSChat()

rewriter_chain = RunnableWithMessageHistory(
    rewrite_prompt | llm | StrOutputParser(),
    get_session_history,
    input_messages_key="question",
    history_messages_key="chat_history"
)

qa_chain_default = answer_prompt_default | llm | StrOutputParser()
qa_chain_code    = answer_prompt_code    | llm | StrOutputParser()

# ===== 메인 답변 함수 =====
def answer(question: str, session_id: str) -> dict:
    # 1) 후속질문 -> 독립질문
    standalone_q = rewriter_chain.invoke(
        {"question": question},
        config={"configurable": {"session_id": session_id}}
    )

    # 2) RAG
    docs = retriever.get_relevant_documents(standalone_q)
    context = "\n\n".join(d.page_content for d in docs[:3])

    # 3) LLM 호출
    if standalone_q.lower().startswith("코드"):
        raw = qa_chain_code.invoke({"context": context, "question": standalone_q})
    else:
        candidates = [
            qa_chain_default.invoke({"context": context, "question": standalone_q})
            for _ in range(3)
        ]
        raw = Counter(candidates).most_common(1)[0][0]

    # 4) JSON normalize
    try:
        res = json.loads(raw)
    except json.JSONDecodeError:
        return {"summary": raw, "details": {}}

    summary = res.get("summary", "")
    details = res.get("details", {})
    if not isinstance(details, dict):
        details = {"response": details}

    return {"summary": summary, "details": details}