use anyhow::Result;

pub trait Renderer {
    // fn start(&mut self) {} // 打开窗口/连接/记录流
    // fn shutdown(&mut self) {} // 收尾
}

pub trait AttachFrom<T> {
    fn attach_from(self, from: &mut T) -> Result<()>;
}
